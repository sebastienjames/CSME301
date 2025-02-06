from expand import expand
from collections import deque, defaultdict
import heapq

def a_star_search (dis_map, time_map, start, end):
	pq = []
	heapq.heappush(pq, (0 + dis_map[start][end], 0 + dis_map[start][end], 0, start, [start], 0))
	visited = set()

	order = 0
		
	while pq:
		_, h, g, node, path, _ = heapq.heappop(pq)

		if node == end:
			return path
		
		if node in visited:
			continue
		
		visited.add(node)
		

		for neighbor in expand(node, time_map):
			if neighbor not in visited:
				g_new = g + time_map[node][neighbor]
				h = dis_map[neighbor][end]
				f = g_new + h
				order += 1

				#if neighbor not in pq or g_new < dis_map[neighbor]:
				heapq.heappush(pq, (f, h, g_new, neighbor, path + [neighbor], order))

	return None


def depth_first_search(time_map, start, end):
	stack = [(start, [start])]
	visited = set()

	if start == end:
		return [start]
	
	while stack:
		node, path = stack.pop()

		if node == end:
			return path
		
		if node not in visited:
			visited.add(node)

		for neighbor in reversed(expand(node, time_map)):
			if neighbor not in visited:
				visited.add(neighbor)
				stack.append((neighbor, path + [neighbor]))

	return None


def breadth_first_search(time_map, start, end):
	queue = deque([(start,[start])])
	visited = set()

	if start == end:
		return [start]

	while queue:
		node, path = queue.popleft()

		if node == end:
			return path
		
		if node not in visited:
			visited.add(node)

		for neighbor in expand(node, time_map):
			if neighbor not in visited:

				visited.add(neighbor)
				queue.append((neighbor, path + [neighbor]))

	return None