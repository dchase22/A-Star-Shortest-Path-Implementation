import heapq


# Define the grid (0 = free, 1 = obstacle)
grid = [
    [0, 0, 0, 0, 1],
    [1, 1, 0, 0, 1],
    [0, 0, 0, 0, 1],
    [1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

# Start and goal positions
start = (0, 0)
goal = (4, 4)

# Movement directions (up, down, left, right)
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(a_grid, a_start, a_goal):
    rows, cols = len(a_grid), len(a_grid[0])
    open_set = []
    heapq.heappush(open_set, (0, a_start))  # (f_score, position)

    came_from = {}  # For reconstructing the path
    g_score = {a_start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        # If goal reached, reconstruct path
        if current == a_goal:
            return reconstruct_path(came_from, current)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            # Skip invalid or blocked cells
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            if a_grid[neighbor[0]][neighbor[1]] == 1:
                continue

            # Cost from start to neighbor
            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, a_goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None  # No path found


def reconstruct_path(came_from, current):
    the_path = [current]
    while current in came_from:
        current = came_from[current]
        the_path.append(current)
    the_path.reverse()
    return the_path


path = a_star_search(grid, start, goal)

if path:
    print("Path found:")
    print(path)
else:
    print("No path found.")

if __name__ == '__main__':
    pass
