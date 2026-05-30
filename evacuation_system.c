#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>

/* ─── Constants ──────────────────────────────────────────── */
#define MAX_NODES   100
#define MAX_NAME    32
#define INF         INT_MAX
#define BLOCKED     -1
#define LOG_FILE    "evacuation_log.txt"
#define GRAPH_FILE  "graph_data.txt"

/* ─── Colour codes (ANSI) ────────────────────────────────── */
#define RED     "\033[1;31m"
#define GREEN   "\033[1;32m"
#define YELLOW  "\033[1;33m"
#define CYAN    "\033[1;36m"
#define BOLD    "\033[1m"
#define RESET   "\033[0m"

/* ─── Graph structures ───────────────────────────────────── */
typedef struct {
    int  weight;       /* base distance + crowd penalty; BLOCKED = -1 */
    int  base_dist;    /* pure physical distance          */
    int  crowd;        /* crowd penalty added dynamically */
} Edge;

typedef struct {
    char name[MAX_NAME];
    int  is_exit;
    int  is_priority_exit;  /* emergency / VIP exit */
} Node;

/* ─── Globals ────────────────────────────────────────────── */
static Edge  graph[MAX_NODES][MAX_NODES];
static Node  nodes[MAX_NODES];
static int   n          = 0;    /* number of nodes     */
static int   graphReady = 0;

/* ─── Utility ────────────────────────────────────────────── */
static void clear_screen(void) {
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

static void pause_enter(void) {
    printf("\nPress ENTER to continue...");
    while (getchar() != '\n');
    getchar();
}

static void log_event(const char *msg) {
    FILE *f = fopen(LOG_FILE, "a");
    if (!f) return;
    time_t t = time(NULL);
    char *ts = ctime(&t);
    ts[strlen(ts)-1] = '\0';   /* remove trailing newline */
    fprintf(f, "[%s] %s\n", ts, msg);
    fclose(f);
}

static int isValid(int v) { return (v >= 0 && v < n); }

/* ─── 1. Graph input ─────────────────────────────────────── */
static void init_graph(void) {
    for (int i = 0; i < MAX_NODES; i++)
        for (int j = 0; j < MAX_NODES; j++) {
            graph[i][j].weight    = 0;
            graph[i][j].base_dist = 0;
            graph[i][j].crowd     = 0;
        }
}

static void input_graph(void) {
    int edges;
    init_graph();

    printf(CYAN "\n+------------------------------+\n");
    printf(    "|    CREATE BUILDING GRAPH     |\n");
    printf(    "+------------------------------+\n" RESET);

    printf("Number of nodes (rooms/areas): ");
    if (scanf("%d", &n) != 1 || n <= 0 || n >= MAX_NODES) {
        printf(RED "Invalid node count!\n" RESET);
        n = 0; return;
    }

    /* Node names & exit flags */
    printf("\nEnter node details:\n");
    for (int i = 0; i < n; i++) {
        printf("  Node %d name (e.g. Lobby, Room-A): ", i);
        scanf("%31s", nodes[i].name);
        printf("  Is node %d an EXIT? (1=Yes, 0=No): ", i);
        scanf("%d", &nodes[i].is_exit);
        nodes[i].is_priority_exit = 0;
        if (nodes[i].is_exit) {
            printf("  Is it a PRIORITY exit? (1=Yes, 0=No): ");
            scanf("%d", &nodes[i].is_priority_exit);
        }
    }

    printf("\nNumber of edges (paths): ");
    scanf("%d", &edges);

    printf("\nEnter each edge as:  u  v  distance  crowd_penalty\n");
    printf("  (Use distance=-1 to mark a BLOCKED path)\n\n");

    int ok = 0;
    while (ok < edges) {
        int u, v, dist, crowd;
        printf("  Edge %d: ", ok+1);
        if (scanf("%d %d %d %d", &u, &v, &dist, &crowd) != 4) {
            printf(RED "  Bad input, try again.\n" RESET);
            continue;
        }
        if (!isValid(u) || !isValid(v)) {
            printf(RED "  Node out of range [0,%d), retry.\n" RESET, n);
            continue;
        }

        if (dist == BLOCKED) {
            graph[u][v].weight = graph[v][u].weight = BLOCKED;
            graph[u][v].base_dist = graph[v][u].base_dist = BLOCKED;
            printf(RED "  Path %d↔%d BLOCKED.\n" RESET, u, v);
        } else {
            if (crowd < 0) crowd = 0;
            int w = dist + crowd;
            graph[u][v].weight    = graph[v][u].weight    = w;
            graph[u][v].base_dist = graph[v][u].base_dist = dist;
            graph[u][v].crowd     = graph[v][u].crowd     = crowd;
            if (crowd > 0)
                printf(YELLOW "  Path %d(%s)↔%d(%s): dist=%d crowd=%d total=%d\n" RESET,
                       u, nodes[u].name, v, nodes[v].name, dist, crowd, w);
            else
                printf(GREEN  "  Path %d(%s)↔%d(%s): dist=%d (clear)\n" RESET,
                       u, nodes[u].name, v, nodes[v].name, dist);
        }
        ok++;
    }

    graphReady = 1;
    log_event("Graph created/updated.");
    printf(GREEN "\n✔ Graph ready (%d nodes, %d edges).\n" RESET, n, edges);
}

/* ─── 2. BFS reachability ────────────────────────────────── */
static int bfs_reachable(int src, int dst) {
    int visited[MAX_NODES] = {0};
    int queue[MAX_NODES], front = 0, back = 0;
    queue[back++] = src;
    visited[src]  = 1;
    while (front < back) {
        int u = queue[front++];
        if (u == dst) return 1;
        for (int v = 0; v < n; v++) {
            if (!visited[v] && graph[u][v].weight > 0) {
                visited[v] = 1;
                queue[back++] = v;
            }
        }
    }
    return 0;
}

/* ─── 3. Dijkstra ────────────────────────────────────────── */
static int minDist(int dist[], int vis[]) {
    int mn = INF, idx = -1;
    for (int i = 0; i < n; i++)
        if (!vis[i] && dist[i] < mn) { mn = dist[i]; idx = i; }
    return idx;
}

static void run_dijkstra(int src) {
    /* Collect exits */
    int exits[MAX_NODES], ec = 0;
    for (int i = 0; i < n; i++)
        if (nodes[i].is_exit) exits[ec++] = i;

    if (ec == 0) { printf(RED "No exits defined!\n" RESET); return; }

    int dist[MAX_NODES], vis[MAX_NODES], par[MAX_NODES];
    for (int i = 0; i < n; i++) {
        dist[i] = INF; vis[i] = 0; par[i] = -1;
    }
    dist[src] = 0;

    for (int iter = 0; iter < n-1; iter++) {
        int u = minDist(dist, vis);
        if (u == -1) break;
        vis[u] = 1;
        for (int v = 0; v < n; v++) {
            int w = graph[u][v].weight;
            if (!vis[v] && w > 0 && dist[u] != INF && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                par[v]  = u;
            }
        }
    }

    /* Pick best exit: priority exit first, then nearest */
    int best = -1, best_d = INF;
    /* Phase 1: prefer priority exits */
    for (int i = 0; i < ec; i++) {
        int e = exits[i];
        if (nodes[e].is_priority_exit && dist[e] < best_d) {
            best_d = dist[e]; best = e;
        }
    }
    /* Phase 2: fallback to any reachable exit */
    if (best == -1) {
        for (int i = 0; i < ec; i++) {
            int e = exits[i];
            if (dist[e] < best_d) { best_d = dist[e]; best = e; }
        }
    }

    printf(CYAN "\n+----------------------------------+\n");
    printf(     "|      EVACUATION RESULT           |\n");
    printf(     "+----------------------------------+\n" RESET);

    if (best == -1 || best_d == INF) {
        printf(RED "✘ No safe path to any exit from %s!\n" RESET, nodes[src].name);
        log_event("Evacuation failed: no path found.");
        return;
    }

    printf(GREEN "✔ Best Exit  : [%d] %s%s\n" RESET,
           best, nodes[best].name,
           nodes[best].is_priority_exit ? " ★(PRIORITY)" : "");
    printf("  Total Cost : %d\n", best_d);

    /* Reconstruct path */
    int path[MAX_NODES], k = 0, cur = best;
    while (cur != -1) { path[k++] = cur; cur = par[cur]; }

    printf("  Route      : ");
    int has_crowd = 0, has_blocked_alt = 0;
    for (int i = k-1; i >= 0; i--) {
        int node = path[i];
        if (nodes[node].is_exit && node == best)
            printf(GREEN "%s[EXIT]" RESET, nodes[node].name);
        else
            printf(BOLD "%s" RESET, nodes[node].name);
        if (i != 0) {
            int nxt = path[i-1];
            if (graph[node][nxt].crowd > 0) {
                printf(YELLOW " ~(%d)~> " RESET, graph[node][nxt].weight);
                has_crowd = 1;
            } else {
                printf(" -(%d)-> ", graph[node][nxt].weight);
            }
        }
    }
    printf("\n");

    /* Reason */
    printf("  Reason     : ");
    if (has_blocked_alt && has_crowd)
        printf("Blocked & crowded paths avoided; safest alternate selected.\n");
    else if (has_crowd)
        printf("Crowd-weighted path used; less-congested edges preferred.\n");
    else
        printf("Optimal shortest path selected.\n");

    /* Show all exit distances */
    printf("\n  Distance to ALL exits:\n");
    for (int i = 0; i < ec; i++) {
        int e = exits[i];
        if (dist[e] == INF)
            printf("    [%d] %-15s : " RED "UNREACHABLE\n" RESET, e, nodes[e].name);
        else
            printf("    [%d] %-15s : %d%s\n", e, nodes[e].name, dist[e],
                   (e == best) ? GREEN " ← chosen" RESET : "");
    }

    /* Log result */
    char logbuf[256];
    snprintf(logbuf, sizeof(logbuf),
             "Evacuation from %s -> %s, cost=%d", nodes[src].name, nodes[best].name, best_d);
    log_event(logbuf);
}

/* ─── 4. Dynamic hazard update ───────────────────────────── */
static void update_hazard(void) {
    if (!graphReady) { printf(RED "Build graph first.\n" RESET); return; }

    printf(CYAN "\n── DYNAMIC HAZARD UPDATE ──\n" RESET);
    int u, v, action;
    printf("  Edge (u v): ");
    scanf("%d %d", &u, &v);
    if (!isValid(u) || !isValid(v)) { printf(RED "Invalid nodes.\n" RESET); return; }

    printf("  Action: 1=Block  2=Set crowd penalty  3=Clear/Restore\n  > ");
    scanf("%d", &action);

    if (action == 1) {
        graph[u][v].weight = graph[v][u].weight = BLOCKED;
        printf(RED "Path %d↔%d is now BLOCKED.\n" RESET, u, v);
        log_event("Path blocked dynamically.");
    } else if (action == 2) {
        int cp;
        printf("  New crowd penalty: ");
        scanf("%d", &cp);
        graph[u][v].crowd     = graph[v][u].crowd     = cp;
        graph[u][v].weight    = graph[v][u].weight
            = graph[u][v].base_dist + cp;
        printf(YELLOW "Crowd penalty on %d↔%d set to %d (total weight=%d).\n" RESET,
               u, v, cp, graph[u][v].weight);
        log_event("Crowd penalty updated.");
    } else if (action == 3) {
        graph[u][v].weight    = graph[v][u].weight    = graph[u][v].base_dist;
        graph[u][v].crowd     = graph[v][u].crowd     = 0;
        printf(GREEN "Path %d↔%d restored to base distance %d.\n" RESET,
               u, v, graph[u][v].base_dist);
        log_event("Path restored.");
    } else {
        printf(RED "Unknown action.\n" RESET);
    }
}

/* ─── 5. Show graph ──────────────────────────────────────── */
static void show_graph(void) {
    if (!graphReady) { printf(RED "No graph loaded.\n" RESET); return; }

    printf(CYAN "\n── ADJACENCY MATRIX ──\n" RESET);
    printf("     ");
    for (int j = 0; j < n; j++) printf("%6d", j);
    printf("\n     ");
    for (int j = 0; j < n; j++) printf("------");
    printf("\n");

    for (int i = 0; i < n; i++) {
        printf("%3d |", i);
        for (int j = 0; j < n; j++) {
            int w = graph[i][j].weight;
            if (w == BLOCKED)      printf(RED  "    X " RESET);
            else if (w == 0)       printf("    . ");
            else if (graph[i][j].crowd > 0)
                                   printf(YELLOW "%5d " RESET, w);
            else                   printf(GREEN  "%5d " RESET, w);
        }
        printf("  %s", nodes[i].name);
        if (nodes[i].is_exit) printf(CYAN " [EXIT%s]" RESET,
                                     nodes[i].is_priority_exit ? "★" : "");
        printf("\n");
    }
    printf("\n  Legend: " GREEN "green" RESET "=clear  " YELLOW "yellow" RESET
           "=crowded  " RED "X" RESET "=blocked\n");
}

/* ─── 6. Reachability report ─────────────────────────────── */
static void reachability_report(void) {
    if (!graphReady) { printf(RED "No graph loaded.\n" RESET); return; }
    int src;
    printf("Source node: ");
    scanf("%d", &src);
    if (!isValid(src)) { printf(RED "Invalid.\n" RESET); return; }

    printf(CYAN "\n── REACHABILITY FROM %s ──\n" RESET, nodes[src].name);
    int unreachable = 0;
    for (int v = 0; v < n; v++) {
        if (v == src) continue;
        int r = bfs_reachable(src, v);
        printf("  %s -> %-15s : %s\n",
               nodes[src].name, nodes[v].name,
               r ? GREEN "Reachable" RESET : RED "UNREACHABLE" RESET);
        if (!r) unreachable++;
    }
    printf("\n  Total unreachable from %s: %d node(s)\n", nodes[src].name, unreachable);
}

/* ─── 7. Save & Load ─────────────────────────────────────── */
static void save_graph(void) {
    if (!graphReady) { printf(RED "No graph to save.\n" RESET); return; }
    FILE *f = fopen(GRAPH_FILE, "w");
    if (!f) { printf(RED "Cannot open file.\n" RESET); return; }

    fprintf(f, "%d\n", n);
    for (int i = 0; i < n; i++)
        fprintf(f, "%s %d %d\n",
                nodes[i].name, nodes[i].is_exit, nodes[i].is_priority_exit);

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            fprintf(f, "%d %d %d\n",
                    graph[i][j].weight, graph[i][j].base_dist, graph[i][j].crowd);

    fclose(f);
    printf(GREEN "✔ Graph saved to " GRAPH_FILE "\n" RESET);
    log_event("Graph saved to file.");
}

static void load_graph(void) {
    FILE *f = fopen(GRAPH_FILE, "r");
    if (!f) { printf(RED "No saved graph found (" GRAPH_FILE ").\n" RESET); return; }

    fscanf(f, "%d", &n);
    for (int i = 0; i < n; i++)
        fscanf(f, "%31s %d %d",
               nodes[i].name, &nodes[i].is_exit, &nodes[i].is_priority_exit);

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            fscanf(f, "%d %d %d",
                   &graph[i][j].weight, &graph[i][j].base_dist, &graph[i][j].crowd);

    fclose(f);
    graphReady = 1;
    printf(GREEN "✔ Graph loaded from " GRAPH_FILE "\n" RESET);
    log_event("Graph loaded from file.");
}

/* ─── 8. Statistics ──────────────────────────────────────── */
static void show_statistics(void) {
    if (!graphReady) { printf(RED "No graph.\n" RESET); return; }

    int edge_count = 0, blocked_count = 0, crowded_count = 0;
    long long total_weight = 0;

    for (int i = 0; i < n; i++)
        for (int j = i+1; j < n; j++) {
            int w = graph[i][j].weight;
            if (w == BLOCKED) { blocked_count++; }
            else if (w > 0) {
                edge_count++;
                total_weight += w;
                if (graph[i][j].crowd > 0) crowded_count++;
            }
        }

    int exit_count = 0, priority_count = 0;
    for (int i = 0; i < n; i++) {
        if (nodes[i].is_exit) exit_count++;
        if (nodes[i].is_priority_exit) priority_count++;
    }

    printf(CYAN "\n+------------------------------+\n");
    printf(     "|      GRAPH STATISTICS        |\n");
    printf(     "+------------------------------+\n" RESET);
    printf("  Nodes           : %d\n", n);
    printf("  Active edges    : %d\n", edge_count);
    printf("  Blocked paths   : " RED "%d\n" RESET, blocked_count);
    printf("  Crowded paths   : " YELLOW "%d\n" RESET, crowded_count);
    printf("  Total exits     : %d  (priority: %d)\n", exit_count, priority_count);
    if (edge_count > 0)
        printf("  Avg edge weight : %.1f\n", (double)total_weight / edge_count);
}

/* ─── 9. Help ────────────────────────────────────────────── */
static void show_help(void) {
    printf(CYAN "\n── HOW TO USE ──\n" RESET);
    printf("  1. Enter Graph     → Define building layout: rooms and corridors\n");
    printf("  2. Show Graph      → View adjacency matrix with colour coding\n");
    printf("  3. Find Safe Exit  → Run Dijkstra from your location\n");
    printf("  4. Update Hazard   → Block/crowd/restore a path in real time\n");
    printf("  5. Reachability    → Check which nodes are reachable via BFS\n");
    printf("  6. Statistics      → Summary of graph state\n");
    printf("  7. Save Graph      → Write graph to " GRAPH_FILE "\n");
    printf("  8. Load Graph      → Read graph from " GRAPH_FILE "\n");
    printf("  9. Help            → This screen\n");
    printf("  0. Exit\n\n");
    printf("  Edge weight = base_distance + crowd_penalty\n");
    printf("  Use distance=-1 to mark a blocked path.\n");
    printf("  Priority exits are chosen first even if slightly farther.\n");
}

/* ─── Main menu ──────────────────────────────────────────── */

/* ─── Browser launcher ───────────────────────────────────────── */
#ifdef _WIN32
#include <windows.h>
#endif

static void launch_frontend(void) {
#ifdef _WIN32
    char exe_dir[512] = {0};
    char cmd[600]     = {0};
    GetModuleFileNameA(NULL, exe_dir, sizeof(exe_dir));
    /* strip filename, keep directory */
    char *sep = strrchr(exe_dir, '\\');
    if (sep) *(sep+1) = '\0';
    snprintf(cmd, sizeof(cmd),
             "start \"\" \"%sevacuation_frontend.html\"", exe_dir);
    system(cmd);
#elif defined(__APPLE__)
    system("open evacuation_frontend.html 2>/dev/null || open ./evacuation_frontend.html");
#else
    system("xdg-open evacuation_frontend.html 2>/dev/null &");
#endif
}

int main(void) {
    int choice;
    log_event("=== System started ===");

    /* Launch browser */
    printf("\n");
    printf("  +------------------------------------------+\n");
    printf("  |   Launching frontend in your browser...  |\n");
    printf("  | (Keep this window open while using app)  |\n");
    printf("  +------------------------------------------+\n\n");
    launch_frontend();
    printf("  If browser did not open, double-click:\n");
    printf("  evacuation_frontend.html\n\n");

    while (1) {
        printf(CYAN "\n+------------------------------------------+\n");
        printf(     "|     SMART CROWD EVACUATION SYSTEM        |\n");
        printf(     "+------------------------------------------+\n" RESET);
        printf("| 1. Enter / Rebuild Graph                 |\n");
        printf("| 2. Show Graph (Matrix)                   |\n");
        printf("| 3. Find Safe Evacuation Path             |\n");
        printf("| 4. Update Hazard (Block / Crowd / Clear) |\n");
        printf("| 5. Reachability Report (BFS)             |\n");
        printf("| 6. Statistics                            |\n");
        printf("| 7. Save Graph to File                    |\n");
        printf("| 8. Load Graph from File                  |\n");
        printf("| 9. Help                                  |\n");
        printf("| 0. Exit                                  |\n");
        printf(CYAN "+------------------------------------------+\n" RESET);
        printf("  Choice: ");
        if (scanf("%d", &choice) != 1) { while(getchar()!='\n'); continue; }

        switch (choice) {
            case 1: input_graph();          break;
            case 2: show_graph();           break;
            case 3:
                if (!graphReady) { printf(RED "Build graph first.\n" RESET); break; }
                {
                    int src;
                    printf("Your current location (node number): ");
                    scanf("%d", &src);
                    if (!isValid(src)) { printf(RED "Invalid node.\n" RESET); break; }
                    run_dijkstra(src);
                }
                break;
            case 4: update_hazard();        break;
            case 5: reachability_report();  break;
            case 6: show_statistics();      break;
            case 7: save_graph();           break;
            case 8: load_graph();           break;
            case 9: show_help();            break;
            case 0:
                printf(GREEN "Exiting safely. Stay safe!\n" RESET);
                log_event("=== System exited ===");
                return 0;
            default:
                printf(RED "Invalid choice.\n" RESET);
        }
    }
}
