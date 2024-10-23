from queue import PriorityQueue
import numpy as np
from collections import deque

def reconstructPath(visited, start, end):
    path = []
    if end in visited:
        node = end
        while node is not None:
            path.append(node)
            node = visited[node] # Truy vết
        path.reverse()  # Đảo ngược để có đường đi từ start đến end
    return path
#-------------------------------------------------------------------------
# def BFS(matrix, start, end):
#     #TODO:
#     path = []
#     visited = {start: None}  # Đánh dấu node bắt đầu là đã được thăm, và nó không có node trước đó
#     queue = deque([start])   # Hàng đợi BFS bắt đầu từ node start

#     while queue:
#         # Lấy node đầu tiên trong hàng đợi
#         currentNode = queue.popleft()
        
#         # Dừng khi node hiện tại là node đích
#         if currentNode == end:
#             break

#         # Duyệt qua tất cả các node kề với currentNode
#         for i in range(len(matrix)):
#             # Kiểm tra nếu node i kề với currentNode và chưa được thăm
#             if matrix[currentNode][i] != 0 and i not in visited:
#                 queue.append(i)  # Thêm node kề vào hàng đợi để thăm tiếp
#                 visited[i] = currentNode  # Lưu lại node trước đó là currentNode
    
#     # Truy vết lại đường đi từ start đến end
#     print(visited)
#     path = reconstructPath(visited, start, end)

#     return visited, path
#-------------------------------------------------------------------------
# def DFS(matrix, start, end):
#     # TODO
#     path = []
#     visited = {start: None}
#     stack = [start]

#     while stack:
#         # Lấy node hiện tại từ stack (DFS đi theo chiều sâu nên pop từ cuối ngăn xếp)
#         currentNode = stack.pop()

#         # Nếu node hiện tại là node đích (end) thì dừng
#         if currentNode == end:
#             break

#         # Duyệt qua tất cả các node kề của currentNode
#         for i in range(len(matrix)):
#             # Kiểm tra xem có đường nối (matrix[currentNode][i] != 0)
#             # và node i chưa được thăm (i not in visited)
#             if matrix[currentNode][i] != 0 and i not in visited:
#                 visited[i] = currentNode
#                 stack.append(i)

#     # Xây dựng lại đường đi từ start đến end
#     path = reconstructPath(visited, start, end)
    
#     return visited, path
#-------------------------------------------------------------------------
# def UCS(matrix, start, end):
#     # TODO:
#     path = []
#     visited = {start: None}  # Khởi tạo dictionary lưu node đã thăm
#     cost = {start: 0}  # Chi phí từ start đến mỗi node

#     # Khởi tạo hàng đợi ưu tiên
#     queue = PriorityQueue()
#     queue.put((0, start))  # Đưa vào hàng đợi với trọng số 0

#     while not queue.empty():
#         # Lấy node có chi phí thấp nhất
#         currentWeight, currentNode = queue.get()
        
#         # Dừng lại khi đến đích
#         if currentNode == end:
#             break
        
#         # Duyệt các node kề
#         for i in range(len(matrix[currentNode])):
#             if matrix[currentNode][i] != 0:  # Có đường đi đến node i
#                 newCost = currentWeight + matrix[currentNode][i]
                
#                 # Nếu chưa thăm hoặc tìm được đường đi tốt hơn
#                 if i not in cost or newCost < cost[i]:
#                     cost[i] = newCost
#                     visited[i] = currentNode
#                     queue.put((newCost, i))  # Thêm vào hàng đợi với trọng số mới

#     # Trả về visited và đường đi từ start đến end      
#     path = reconstructPath(visited, start, end)
    
#     return visited, path


# def UCS(matrix, start, end):
#     # TODO:  
#     path = []
#     visited = {}

#     queue = []
#     queue.append([0, start])
#     visited.update({start: None})
#     visit={}
#     prev=start
#     visited_temp={}

#     while (len(queue)):
#         queue = sorted(queue) # priority queue
#         node = queue[-1]
#         del queue[-1]
#         prev = node[1]
#         # node[0] *= -1

#         if (node[1] == end):
#             del queue[-1]
#             queue = sorted(queue)
#             break  

#         if (node[1] not in visit):
#             for i in range(len(matrix[node[1]])):
#                 if(matrix[node[1]][i] != 0):
#                     queue.append([node[0] + matrix[node[1]][i] *- 1, i])
#                     visited.update({i: node})
#                     visited_temp[i] = prev
        
#         visit[node[1]] = 1

#     path = reconstructPath(visited_temp, start, end)
#     return visited, path

#-------------------------------------------------------------------------

def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path = []
    visited = {start: None}  # Tạo dictionary để lưu các nút đã thăm
    stack = [(start, 0)]  # Sử dụng stack để lưu trữ các nút cần thăm, bao gồm trọng số
    
    while stack:
        # Sắp xếp các nút trong stack theo trọng số (chi phí heuristic)
        stack.sort(key=lambda x: matrix[x[0]][end])  # Giả định matrix chứa trọng số từ nút đến đích
        curentNode, _ = stack.pop(0)  # Lấy nút đầu tiên trong danh sách đã sắp xếp
        
        if curentNode == end:
            break  # Dừng nếu đã đến nút kết thúc

        for i in range(len(matrix)):
            if matrix[curentNode][i] != 0 and i not in visited:
                # Chỉ thêm vào stack nếu chưa được thăm
                stack.append((i, matrix[i][end]))  # Thêm nút cùng với trọng số heuristic
                visited[i] = curentNode  # Ghi nhận nút cha của nút hiện tại

    path = reconstructPath(visited, start, end)
    return visited, path

#-------------------------------------------------------------------------
if __name__ == '__main__':
    matrix = []
    with open('input01.txt', 'r', encoding='utf-8') as data:
        start, end = data.readline().strip().split(" ")  # Đọc dòng đầu tiên và phân tách
        start, end = int(start), int(end)  # Chuyển đổi sang số nguyên nếu cần

        for line in data:
            items = line.rstrip().split(" ")
            matrix.append([int(item) for item in items]) 

    visited, path = GBFS(matrix, start, end);
    print("path:", path)
    print("visited:", visited)




# def Astar(matrix, start, end, pos):
#     """
#     A* Search algorithm
#     heuristic: eclid distance based positions parameter
#      Parameters:
#     ---------------------------
#     matrix: np array UCS
#         The graph's adjacency matrix
#     start: integer 
#         starting node
#     end: integer
#         ending node
#     pos: dictionary. keys are nodes, values are positions
#         positions of graph nodes
#     Returns
#     ---------------------
#     visited
#         The dictionary contains visited nodes: each key is a visited node, 
#         each value is the key's adjacent node which is visited before key.
#     path: list
#         Founded path
#     """
#     # TODO: 

#     path=[]
#     visited={}
#     return visited, path

