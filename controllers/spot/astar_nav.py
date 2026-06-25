import math
import heapq

class AStarNavigator:
    """Simple grid-based A* for waypoint navigation on uneven terrain."""
    def __init__(self, grid_size=80):
        self.grid_size = grid_size

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def find_path(self, start, goal):
        """Returns smoothed path from start to goal (list of (x,y))."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if math.hypot(current[0] - goal[0], current[1] - goal[1]) < 2.0:
                return self._reconstruct_path(came_from, current)

            for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.grid_size and 0 <= neighbor[1] < self.grid_size):
                    continue
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
