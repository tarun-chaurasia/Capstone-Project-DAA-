"""
Mini-Courier Planner  -  DAA Capstone (ETCCDA302)

Given a list of parcels and a road map, this program:
  Phase 2 : sorts parcels by deadline/priority (Merge Sort)
            and picks the best parcels for ONE van trip (0/1 Knapsack DP)
  Phase 3 : finds shortest paths (Dijkstra) and makes the full
            delivery route (Nearest-Neighbor TSP approximation)
  Phase 4 : measures runtime for small -> large inputs and draws a chart

Run:  python courier_planner.py
"""

import csv
import heapq
import os
import random
import time

INF = float("inf")
DEPOT = 0              # node 0 is the courier warehouse (start and end point)
VAN_CAPACITY_KG = 30   # max weight the van can carry in one trip
SPEED_KMPH = 30        # average van speed, used only for the deadline check

HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------
# 1. DATA LOADING  (Phase 1: data design)
# ---------------------------------------------------------------------
def load_parcels(path):
    """Read parcels from CSV. Each parcel is a dictionary."""
    parcels = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            parcels.append({
                "id": row["parcel_id"],
                "dest": int(row["destination"]),     # which node to deliver to
                "weight": int(row["weight_kg"]),
                "priority": int(row["priority"]),    # 1..10, bigger = more important
                "deadline": int(row["deadline_hr"]), # deliver within these hours
            })
    return parcels


def load_graph(path):
    """Read roads from CSV into an adjacency list: {node: [(neighbor, km), ...]}"""
    graph = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            u, v, w = int(row["from"]), int(row["to"]), int(row["distance_km"])
            graph.setdefault(u, []).append((v, w))   # roads work both ways
            graph.setdefault(v, []).append((u, w))
    return graph


# ---------------------------------------------------------------------
# 2. SORTING  (Phase 2)  - Merge Sort  O(n log n)   (+ Bubble Sort for comparison)
# ---------------------------------------------------------------------
def urgency_key(parcel):
    """Smaller deadline first; if same deadline, higher priority first."""
    return (parcel["deadline"], -parcel["priority"])


def merge(left, right, key):
    result = []
    i = j = 0
    while i < len(left) and j < len(right):
        if key(left[i]) <= key(right[j]):
            result.append(left[i])
            i += 1
        else:
            result.append(right[j])
            j += 1
    result.extend(left[i:])
    result.extend(right[j:])
    return result


def merge_sort(items, key=urgency_key):
    """Divide list into halves, sort each half, then merge. O(n log n)."""
    if len(items) <= 1:
        return items
    mid = len(items) // 2
    left = merge_sort(items[:mid], key)
    right = merge_sort(items[mid:], key)
    return merge(left, right, key)


def bubble_sort(items, key=urgency_key):
    """Simple but slow sort, O(n^2). Used ONLY to compare with Merge Sort."""
    arr = items[:]
    n = len(arr)
    for i in range(n):
        swapped = False
        for j in range(n - 1 - i):
            if key(arr[j]) > key(arr[j + 1]):
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
                swapped = True
        if not swapped:          # already sorted -> stop early
            break
    return arr


# ---------------------------------------------------------------------
# 3. DYNAMIC PROGRAMMING  (Phase 2)  - 0/1 Knapsack  O(n * capacity)
# ---------------------------------------------------------------------
def knapsack(parcels, capacity):
    """
    Choose parcels so total weight <= capacity and total priority is maximum.
    dp[i][c] = best priority using the first i parcels with capacity c.
    Returns (best_value, chosen_parcels, dp_table).
    """
    n = len(parcels)
    dp = [[0] * (capacity + 1) for _ in range(n + 1)]

    for i in range(1, n + 1):
        w = parcels[i - 1]["weight"]
        v = parcels[i - 1]["priority"]
        for c in range(capacity + 1):
            dp[i][c] = dp[i - 1][c]                    # option 1: skip this parcel
            if w <= c and dp[i - 1][c - w] + v > dp[i][c]:
                dp[i][c] = dp[i - 1][c - w] + v        # option 2: take it

    # walk backwards through the table to find WHICH parcels were taken
    chosen = []
    c = capacity
    for i in range(n, 0, -1):
        if dp[i][c] != dp[i - 1][c]:
            chosen.append(parcels[i - 1])
            c -= parcels[i - 1]["weight"]
    chosen.reverse()
    return dp[n][capacity], chosen, dp


# ---------------------------------------------------------------------
# 4. DIJKSTRA  (Phase 3)  - shortest path  O((V + E) log V)
# ---------------------------------------------------------------------
def dijkstra(graph, source):
    """Shortest distance from `source` to every node. Returns (dist, prev)."""
    dist = {node: INF for node in graph}
    prev = {}
    dist[source] = 0
    heap = [(0, source)]                       # (distance so far, node)
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:                        # old/outdated entry, skip
            continue
        for v, w in graph[u]:
            if d + w < dist[v]:
                dist[v] = d + w
                prev[v] = u
                heapq.heappush(heap, (dist[v], v))
    return dist, prev


def build_path(prev, source, target):
    """Rebuild the actual road path source -> target using `prev`."""
    path = [target]
    while path[-1] != source:
        path.append(prev[path[-1]])
    path.reverse()
    return path


# ---------------------------------------------------------------------
# 5. NEAREST-NEIGHBOR TSP  (Phase 3)  - approximation  O(k^2)
# ---------------------------------------------------------------------
def nearest_neighbor_route(graph, depot, stops):
    """
    Start at depot, always go to the closest unvisited stop, finally return.
    Distances between stops come from Dijkstra (real road distances).
    Returns (stop_order, total_km, full_road_path).
    """
    nodes = [depot] + sorted(set(stops) - {depot})
    sp = {n: dijkstra(graph, n) for n in nodes}      # Dijkstra from every stop

    unvisited = set(nodes) - {depot}
    order = [depot]
    total = 0
    current = depot
    while unvisited:
        nxt = min(sorted(unvisited), key=lambda x: sp[current][0][x])
        total += sp[current][0][nxt]
        order.append(nxt)
        unvisited.remove(nxt)
        current = nxt
    total += sp[current][0][depot]                   # come back to warehouse
    order.append(depot)

    full_path = [depot]
    for a, b in zip(order, order[1:]):
        full_path.extend(build_path(sp[a][1], a, b)[1:])
    return order, total, full_path, sp


# ---------------------------------------------------------------------
# 6. DEADLINE CHECK  (extra: shows if the route meets the deadlines)
# ---------------------------------------------------------------------
def check_deadlines(parcels, order, sp):
    """Arrival time at each stop = distance travelled so far / speed."""
    arrival = {}
    travelled = 0
    for a, b in zip(order, order[1:]):
        travelled += sp[a][0][b]
        arrival.setdefault(b, travelled / SPEED_KMPH)   # first visit time
    rows = []
    for p in parcels:
        t = arrival[p["dest"]]
        rows.append((p["id"], p["dest"], p["deadline"], t, t <= p["deadline"]))
    return rows


# ---------------------------------------------------------------------
# 7. RANDOM DATA GENERATORS  (for Phase 4 testing with bigger inputs)
# ---------------------------------------------------------------------
def generate_graph(num_nodes, seed=1):
    rnd = random.Random(seed)
    graph = {i: [] for i in range(num_nodes)}
    seen = set()

    def add(u, v):
        key = (min(u, v), max(u, v))
        if u == v or key in seen:
            return
        seen.add(key)
        w = rnd.randint(1, 20)
        graph[u].append((v, w))
        graph[v].append((u, w))

    for i in range(1, num_nodes):            # connect every node to an earlier one
        add(i, rnd.randint(0, i - 1))        # -> graph is always connected
    for _ in range(num_nodes * 2):           # extra roads
        add(rnd.randrange(num_nodes), rnd.randrange(num_nodes))
    return graph


def generate_parcels(n, num_nodes, seed=1):
    rnd = random.Random(seed)
    return [{
        "id": f"P{i + 1:04d}",
        "dest": rnd.randint(1, num_nodes - 1),
        "weight": rnd.randint(1, 10),
        "priority": rnd.randint(1, 10),
        "deadline": rnd.randint(1, 12),
    } for i in range(n)]


# ---------------------------------------------------------------------
# 8. PRINTING HELPERS
# ---------------------------------------------------------------------
def show_parcels(title, parcels):
    print(f"\n{title}")
    print(f"  {'ID':<6}{'Dest':<6}{'Wt(kg)':<8}{'Priority':<10}{'Deadline(hr)'}")
    for p in parcels:
        print(f"  {p['id']:<6}{p['dest']:<6}{p['weight']:<8}{p['priority']:<10}{p['deadline']}")


def show_dp_table(parcels, dp, capacity):
    print("\nDP table  (row = after considering parcel, column = van capacity in kg)")
    print("       " + "".join(f"{c:>3}" for c in range(capacity + 1)))
    for i, row in enumerate(dp):
        label = "start" if i == 0 else parcels[i - 1]["id"]
        print(f"{label:>6} " + "".join(f"{x:>3}" for x in row))


# ---------------------------------------------------------------------
# 9. THE FULL PIPELINE (used for both the demo and the performance test)
# ---------------------------------------------------------------------
def plan_trip(parcels, graph, capacity):
    sorted_parcels = merge_sort(parcels)
    best, chosen, dp = knapsack(sorted_parcels, capacity)
    stops = [p["dest"] for p in chosen]
    order, km, full_path, sp = nearest_neighbor_route(graph, DEPOT, stops)
    return sorted_parcels, best, chosen, dp, order, km, full_path, sp


def demo():
    print("=" * 66)
    print("  MINI-COURIER PLANNER  -  DAA Capstone (ETCCDA302)")
    print("=" * 66)

    parcels = load_parcels(os.path.join(HERE, "data", "sample_parcels.csv"))
    graph = load_graph(os.path.join(HERE, "data", "sample_graph.csv"))
    show_parcels(f"All parcels at warehouse ({len(parcels)} parcels, "
                 f"{sum(p['weight'] for p in parcels)} kg in total)", parcels)
    print(f"\nVan capacity = {VAN_CAPACITY_KG} kg")

    # ---------------- Phase 2 ----------------
    print("\n" + "-" * 66 + "\nPHASE 2: Prioritization and box filling\n" + "-" * 66)
    sorted_parcels = merge_sort(parcels)
    show_parcels("After Merge Sort (earliest deadline first, then higher priority):",
                 sorted_parcels)

    best, chosen, dp = knapsack(sorted_parcels, VAN_CAPACITY_KG)
    show_dp_table(sorted_parcels, dp, VAN_CAPACITY_KG)
    show_parcels(f"Parcels loaded in the van (total priority = {best}, "
                 f"total weight = {sum(p['weight'] for p in chosen)} kg):", chosen)
    left_behind = [p for p in sorted_parcels if p not in chosen]
    print("\nLeft for the next trip:", ", ".join(p["id"] for p in left_behind))

    # ---------------- Phase 3 ----------------
    print("\n" + "-" * 66 + "\nPHASE 3: Route maker and graph processing\n" + "-" * 66)
    print("\nGraph (adjacency list)  node: [(neighbor, km), ...]")
    for node in sorted(graph):
        print(f"  {node}: {graph[node]}")

    stops = sorted({p["dest"] for p in chosen})
    dist0, prev0 = dijkstra(graph, DEPOT)
    print(f"\nDijkstra - shortest distance from warehouse (node {DEPOT}):")
    for s in stops:
        print(f"  to node {s}: {dist0[s]} km   path = {build_path(prev0, DEPOT, s)}")

    order, km, full_path, sp = nearest_neighbor_route(graph, DEPOT, stops)
    print("\nNearest-Neighbor TSP result")
    print("  Stop order     :", " -> ".join(map(str, order)))
    print("  Full road path :", " -> ".join(map(str, full_path)))
    print(f"  Total route length = {km} km")

    print("\nDeadline check (van speed = %d km/h):" % SPEED_KMPH)
    print(f"  {'Parcel':<8}{'Node':<6}{'Deadline':<10}{'Arrival(hr)':<13}Status")
    for pid, node, dl, t, ok in check_deadlines(chosen, order, sp):
        print(f"  {pid:<8}{node:<6}{dl:<10}{t:<13.2f}{'ON TIME' if ok else 'LATE'}")


# ---------------------------------------------------------------------
# 10. PHASE 4 : PERFORMANCE TESTING
# ---------------------------------------------------------------------
def timed(func, *args, reps=3):
    """Run func a few times and return the best time in milliseconds."""
    best = INF
    result = None
    for _ in range(reps):
        start = time.perf_counter()
        result = func(*args)
        best = min(best, time.perf_counter() - start)
    return best * 1000, result


def performance_test(sizes=(10, 25, 50, 100, 200, 500, 1000)):
    print("\n" + "=" * 66 + "\nPHASE 4: Performance testing\n" + "=" * 66)
    rows = []
    for n in sizes:
        num_nodes = max(10, n // 2) + 1
        graph = generate_graph(num_nodes)
        parcels = generate_parcels(n, num_nodes)
        capacity = max(50, n)             # bigger input -> bigger van, so many stops

        t_merge, sorted_parcels = timed(merge_sort, parcels)
        t_bubble, bubble_result = timed(bubble_sort, parcels)
        assert [urgency_key(p) for p in sorted_parcels] == \
               [urgency_key(p) for p in bubble_result], "sorting mismatch!"
        t_knap, (best, chosen, _) = timed(knapsack, sorted_parcels, capacity)
        stops = [p["dest"] for p in chosen]
        t_dij, _ = timed(dijkstra, graph, DEPOT)
        t_tsp, (order, km, _, _) = timed(nearest_neighbor_route, graph, DEPOT, stops)

        assert sum(p["weight"] for p in chosen) <= capacity, "van overloaded!"
        rows.append((n, len(chosen), len(set(stops)), t_merge, t_bubble,
                     t_knap, t_dij, t_tsp, km))

    header = (f"{'Parcels':>8}{'Chosen':>8}{'Stops':>7}{'Merge(ms)':>11}{'Bubble(ms)':>12}"
              f"{'Knapsack(ms)':>14}{'1xDijkstra(ms)':>16}{'DijkstraxK+TSP(ms)':>20}{'Route(km)':>11}")
    print("\n" + header)
    for r in rows:
        print(f"{r[0]:>8}{r[1]:>8}{r[2]:>7}{r[3]:>11.3f}{r[4]:>12.3f}"
              f"{r[5]:>14.3f}{r[6]:>16.3f}{r[7]:>20.3f}{r[8]:>11}")
    print("\nAll correctness checks passed (both sorts agree, van never overloaded).")
    return rows


def save_results(rows):
    out_dir = os.path.join(HERE, "results")
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, "runtime_results.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["parcels", "chosen", "stops", "merge_ms", "bubble_ms",
                    "knapsack_ms", "dijkstra_ms", "route_tsp_ms", "route_km"])
        w.writerows(rows)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -> chart skipped (pip install matplotlib)")
        return

    n = [r[0] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))

    ax[0].plot(n, [r[3] for r in rows], "o-", label="Merge Sort  O(n log n)")
    ax[0].plot(n, [r[4] for r in rows], "s-", label="Bubble Sort  O(n^2)")
    ax[0].set_title("Sorting: Merge vs Bubble")

    ax[1].plot(n, [r[5] for r in rows], "o-", label="Knapsack DP  O(n x capacity)")
    ax[1].plot(n, [r[6] for r in rows], "^-", label="Dijkstra (1 run)")
    ax[1].plot(n, [r[7] for r in rows], "s-", label="Dijkstra x stops + NN-TSP")
    ax[1].set_title("DP, Dijkstra and Route planning")

    for a in ax:
        a.set_xlabel("Number of parcels")
        a.set_ylabel("Time (milliseconds)")
        a.set_xscale("log")
        a.set_yscale("log")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    fig.suptitle("Mini-Courier Planner - Runtime vs Input Size")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "runtime_chart.png"), dpi=150)
    print("Saved: results/runtime_chart.png and results/runtime_results.csv")


if __name__ == "__main__":
    demo()
    save_results(performance_test())
