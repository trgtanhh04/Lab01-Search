import numpy as np
from collections import deque
from queue import PriorityQueue

def reconstructPath(visited, start, end):
    path = []
    if end in visited:
        node = end
        while node is not None:
            path.append(node)
            node = visited[node] # Truy vết
        path.reverse()  # Đảo ngược để có đường đi từ start đến end
    return path

def BFS(matrix, start, end):
    #TODO:
    path = []
    visited = {start: None}  # Đánh dấu node bắt đầu là đã được thăm, và nó không có node trước đó
    queue = deque([start])   # Hàng đợi BFS bắt đầu từ node start

    while queue:
        # Lấy node đầu tiên trong hàng đợi
        currentNode = queue.popleft()
        
        # Dừng khi node hiện tại là node đích
        if currentNode == end:
            break

        # Duyệt qua tất cả các node kề với currentNode
        for i in range(len(matrix)):
            # Kiểm tra nếu node i kề với currentNode và chưa được thăm
            if matrix[currentNode][i] != 0 and i not in visited:
                queue.append(i)  # Thêm node kề vào hàng đợi để thăm tiếp
                visited[i] = currentNode  # Lưu lại node trước đó là currentNode
    
    # Truy vết lại đường đi từ start đến end
    path = reconstructPath(visited, start, end)

    return visited, path

def DFS(matrix, start, end):
    # TODO
    path = []
    visited = {start: None}
    stack = [start]

    while stack:
        # Lấy node hiện tại từ stack (DFS đi theo chiều sâu nên pop từ cuối ngăn xếp)
        currentNode = stack.pop()

        # Nếu node hiện tại là node đích (end) thì dừng
        if currentNode == end:
            break

        # Duyệt qua tất cả các node kề của currentNode
        for i in range(len(matrix)):
            # Kiểm tra xem có đường nối (matrix[currentNode][i] != 0)
            # và node i chưa được thăm (i not in visited)
            if matrix[currentNode][i] != 0 and i not in visited:
                visited[i] = currentNode
                stack.append(i)

    # Xây dựng lại đường đi từ start đến end
    path = reconstructPath(visited, start, end)
    
    return visited, path


def UCS(matrix, start, end):
    # TODO:
    path = []
    visited = {start: None}  # Khởi tạo dictionary lưu node đã thăm
    cost = {start: 0}  # Chi phí từ start đến mỗi node

    # Khởi tạo hàng đợi ưu tiên
    queue = PriorityQueue()
    queue.put((0, start))  # Đưa vào hàng đợi với trọng số 0

    while not queue.empty():
        # Lấy node có chi phí thấp nhất
        currentWeight, currentNode = queue.get()
        
        # Dừng lại khi đến đích
        if currentNode == end:
            break
        
        # Duyệt các node kề
        for i in range(len(matrix[currentNode])):
            if matrix[currentNode][i] != 0:  # Có đường đi đến node i
                newCost = currentWeight + matrix[currentNode][i]
                
                # Nếu chưa thăm hoặc tìm được đường đi tốt hơn
                if i not in cost or newCost < cost[i]:
                    cost[i] = newCost
                    visited[i] = currentNode
                    queue.put((newCost, i))  # Thêm vào hàng đợi với trọng số mới

    # Trả về visited và đường đi từ start đến end      
    path = reconstructPath(visited, start, end)
    
    return visited, path



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
    path=[]
    visited={}
    return visited, path

def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 

    path=[]
    visited={}
    return visited, path

