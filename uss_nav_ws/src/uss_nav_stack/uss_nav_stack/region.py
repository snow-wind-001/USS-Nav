from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
import os
import sys
from typing import Dict, Iterable, List, Sequence, Set, Tuple

import networkx as nx

from .common import EDGE_PARENT, SCGEdgeState

for _prefix in (os.environ.get("CONDA_PREFIX"), "/home/a4201/anaconda3/envs/TLCForMer-main"):
    if not _prefix:
        continue
    _site = os.path.join(_prefix, "lib", "python3.10", "site-packages")
    if os.path.isdir(_site) and _site not in sys.path:
        sys.path.append(_site)

try:
    import igraph as ig  # type: ignore
    import leidenalg  # type: ignore

    _HAS_LEIDEN = True
except Exception:
    ig = None
    leidenalg = None
    _HAS_LEIDEN = False


@dataclass
class RegionConfig:
    leiden_resolution: float = 0.02
    forced_edge_epsilon: float = 0.05
    use_leiden: bool = True
    min_region_size: int = 6


def _edge_weight(edge: SCGEdgeState, epsilon: float) -> float:
    return float(epsilon if edge.edge_type == EDGE_PARENT else 1.0)


def _build_weighted_graph(
    node_ids: Sequence[int],
    edges: Sequence[SCGEdgeState],
    epsilon: float,
) -> nx.Graph:
    graph = nx.Graph()
    graph.add_nodes_from(node_ids)
    for edge in edges:
        weight = _edge_weight(edge, epsilon)
        graph.add_edge(int(edge.from_id), int(edge.to_id), weight=weight)
    return graph


def _cluster_with_networkx(node_ids: Sequence[int], edges: Sequence[SCGEdgeState], epsilon: float) -> Dict[int, int]:
    graph = _build_weighted_graph(node_ids, edges, epsilon)
    if graph.number_of_nodes() == 0:
        return {}
    if graph.number_of_edges() == 0:
        return {int(nid): i for i, nid in enumerate(graph.nodes())}
    communities = nx.community.greedy_modularity_communities(graph, weight="weight")
    labels: Dict[int, int] = {}
    for cid, com in enumerate(communities):
        for nid in com:
            labels[int(nid)] = int(cid)
    return labels


def _cluster_with_leiden(
    node_ids: Sequence[int],
    edges: Sequence[SCGEdgeState],
    config: RegionConfig,
) -> Dict[int, int]:
    graph = _build_weighted_graph(node_ids, edges, config.forced_edge_epsilon)
    if graph.number_of_nodes() == 0:
        return {}
    if graph.number_of_edges() == 0:
        return {int(nid): i for i, nid in enumerate(graph.nodes())}
    if not _HAS_LEIDEN:
        return _cluster_with_networkx(node_ids, edges, config.forced_edge_epsilon)

    idx_to_nid = [int(nid) for nid in graph.nodes()]
    nid_to_idx = {nid: idx for idx, nid in enumerate(idx_to_nid)}
    ig_graph = ig.Graph()  # type: ignore[union-attr]
    ig_graph.add_vertices(len(idx_to_nid))
    ig_edges: List[Tuple[int, int]] = []
    ig_weights: List[float] = []
    for u, v, data in graph.edges(data=True):
        ig_edges.append((nid_to_idx[int(u)], nid_to_idx[int(v)]))
        ig_weights.append(float(data.get("weight", 1.0)))
    if ig_edges:
        ig_graph.add_edges(ig_edges)

    partition = leidenalg.find_partition(  # type: ignore[union-attr]
        ig_graph,
        leidenalg.RBConfigurationVertexPartition,  # type: ignore[union-attr]
        weights=ig_weights if ig_edges else None,
        resolution_parameter=max(1e-4, float(config.leiden_resolution)),
    )
    labels: Dict[int, int] = {}
    for cid, community in enumerate(partition):
        for idx in community:
            labels[int(idx_to_nid[int(idx)])] = int(cid)
    return labels


def _suppress_oversegmentation(
    candidate_labels: Dict[int, int],
    graph: nx.Graph,
    min_region_size: int,
) -> Dict[int, int]:
    if min_region_size <= 1 or not candidate_labels:
        return dict(candidate_labels)

    labels = dict(candidate_labels)
    max_rounds = 3
    for _ in range(max_rounds):
        sizes = Counter(labels.values())
        small_regions = [cid for cid, size in sizes.items() if size < min_region_size]
        if not small_regions:
            break
        changed = False
        for cid in small_regions:
            members = [nid for nid, rid in labels.items() if rid == cid]
            if not members:
                continue
            neighbor_scores: Dict[int, float] = defaultdict(float)
            for nid in members:
                for neigh in graph.neighbors(nid):
                    target = labels.get(int(neigh))
                    if target is None or target == cid:
                        continue
                    edge_data = graph.get_edge_data(nid, neigh, default={})
                    neighbor_scores[int(target)] += float(edge_data.get("weight", 1.0))
            if not neighbor_scores:
                continue
            target_region = max(neighbor_scores.items(), key=lambda item: item[1])[0]
            for nid in members:
                labels[nid] = int(target_region)
            changed = True
        if not changed:
            break
    return labels


def cluster_regions(node_ids: Sequence[int], edges: Sequence[SCGEdgeState], config: RegionConfig) -> Dict[int, int]:
    if config.use_leiden and _HAS_LEIDEN:
        return _cluster_with_leiden(node_ids, edges, config)
    return _cluster_with_networkx(node_ids, edges, config.forced_edge_epsilon)


def _collect_candidate_nodes(
    graph: nx.Graph,
    prev_labels: Dict[int, int],
    new_node_ids: Iterable[int],
) -> Set[int]:
    candidate: Set[int] = set(int(x) for x in new_node_ids)
    expanded = list(candidate)
    for nid in expanded:
        if graph.has_node(nid):
            candidate.update(int(x) for x in graph.neighbors(nid))
    for nid in list(candidate):
        rid = prev_labels.get(nid, None)
        if rid is None:
            continue
        candidate.update(int(k) for k, v in prev_labels.items() if int(v) == int(rid))
    return candidate


def _stabilize_region_ids(
    candidate_labels: Dict[int, int],
    prev_labels: Dict[int, int],
) -> Dict[int, int]:
    overlap: Dict[int, Dict[int, int]] = {}
    for nid, cid in candidate_labels.items():
        old = prev_labels.get(nid, None)
        if old is None:
            continue
        overlap.setdefault(cid, {})
        overlap[cid][old] = overlap[cid].get(old, 0) + 1

    remap: Dict[int, int] = {}
    used_old: Set[int] = set()
    next_id = (max(prev_labels.values()) + 1) if prev_labels else 0
    for cid in sorted(set(candidate_labels.values())):
        candidates = overlap.get(cid, {})
        if candidates:
            best_old = max(candidates.items(), key=lambda x: x[1])[0]
            if best_old not in used_old:
                remap[cid] = best_old
                used_old.add(best_old)
                continue
        remap[cid] = next_id
        next_id += 1
    return {nid: remap[cid] for nid, cid in candidate_labels.items()}


def incremental_region_update(
    node_ids: Sequence[int],
    edges: Sequence[SCGEdgeState],
    prev_labels: Dict[int, int],
    new_node_ids: Sequence[int],
    config: RegionConfig,
) -> Dict[int, int]:
    if not prev_labels:
        return cluster_regions(node_ids, edges, config)
    if not new_node_ids:
        return {int(nid): int(prev_labels.get(int(nid), -1)) for nid in node_ids}

    graph = _build_weighted_graph(node_ids, edges, config.forced_edge_epsilon)
    candidate = _collect_candidate_nodes(graph, prev_labels, new_node_ids)
    if not candidate:
        return {int(nid): int(prev_labels.get(int(nid), -1)) for nid in node_ids}

    sub_edges = [
        edge
        for edge in edges
        if int(edge.from_id) in candidate and int(edge.to_id) in candidate
    ]
    candidate_labels = cluster_regions(sorted(candidate), sub_edges, config)
    subgraph = _build_weighted_graph(sorted(candidate), sub_edges, config.forced_edge_epsilon)
    candidate_labels = _suppress_oversegmentation(candidate_labels, subgraph, config.min_region_size)
    stabilized = _stabilize_region_ids(candidate_labels, prev_labels)

    merged: Dict[int, int] = {}
    for nid in node_ids:
        key = int(nid)
        merged[key] = int(stabilized.get(key, prev_labels.get(key, -1)))
    return merged


def leiden_available() -> bool:
    return _HAS_LEIDEN
