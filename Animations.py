from Graphs import initialize, generateFigure, getRawData, readMatrix, np
from Colors import *
from student_functions import DFS, BFS, UCS, GBFS, Astar
import pygame
import matplotlib.pyplot as plt
from pygame.locals import *
pygame.init()
clock=   pygame.time.Clock()
font =   pygame.font.Font(pygame.font.get_default_font(), 25)
fps = 5  # frames per sec
window = pygame.display.set_mode((800, 800), DOUBLEBUF)
screen = pygame.display.get_surface()

time_delay=None

def drawFig(raw_data, size):
    white = (255,255,255)
    screen.fill(white)
    surf = pygame.image.fromstring(raw_data, size, "RGB")
    screen.blit(surf, (0,0))
    pygame.display.flip()

def update(G, color_map, pos, algorithm, t):
   
    fig = generateFigure(G, color_map, pos)
    plt.savefig("%s_%d.eps"%(algorithm,t), dpi=300, bbox_inches='tight')
    raw_data, size= getRawData(fig)
    drawFig(raw_data,size)
    pygame.display.update()
    clock.tick(fps)
    pygame.time.delay(time_delay)

def quit_event():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.display.quit()
            pygame.quit()
            quit()

def searchAnimation(matrix, visited, G, pos, color_map, algorithm, t):
    tmp=[]
    for v1, v2 in visited.items():
        cur_node = v1
        queue_nodes = np.where(matrix[cur_node]!=0)[0]
        color_map[cur_node]=current_color
        for node in queue_nodes:
            if node not in tmp:
                color_map[node]=queue_color
        update(G, pos, color_map, algorithm, t)
        t +=1
        color_map[cur_node]=visited_color
        tmp.append(cur_node)
        update(G, pos, color_map, algorithm, t)
        t+=1


def paintPath(path, G, pos, color_map):
    n_nodes=len(path)
    for i in range(n_nodes):
        node=path[i]
        color_map[node]=path_node_color
        if i< (n_nodes-1):
            G[node][path[i+1]]['color'] = path_color
    
def run(input, algorithm, delay):
    global time_delay
    time_delay=delay
    matrix, start, end=readMatrix(input)
    G, pos, color_map=initialize(matrix)
    t = 1
    update(G, pos, color_map, algorithm, t)
    visited = None
    if algorithm == 'bfs':
        visited, path = BFS(matrix, start, end)
    elif algorithm == 'dfs':
        visited, path = DFS(matrix, start, end)
    elif algorithm == 'ucs':
        visited, path  = UCS(matrix, start, end)
    elif algorithm == 'greedy':
        visited, path  = GBFS(matrix, start, end)
    elif algorithm == 'astar':
        visited, path  = Astar(matrix, start, end, pos)
    else:
        print("Pass a search algorithm to run program.")
    
    t=1

    searchAnimation(matrix, visited, G, pos, color_map, algorithm, t)
    paintPath(path, G, pos, color_map)
    
    update(G, pos, color_map, algorithm, 0)
    while True:
        quit_event()
    


    

