Try:

python pacman.py -l smallMaze -p SearchAgent -a fn=dfs

python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs

python pacman.py -l bigMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

python pacman.py -l bigMaze -p SearchAgent -a fn=greedy,heuristic=euclideanHeuristic