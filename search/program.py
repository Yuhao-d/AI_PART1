# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers

from .core import CellState, Coord, Direction, MoveAction
from .utils import render_board
import heapq
from typing import Callable

def is_valid_move(coord: Coord, board: dict[Coord, CellState]) -> bool:
    """
    Check if a coordinate is valid for movement.
    
    Valid means:
    - Within board boundaries (0-7 rows/cols)
    - Exists in board data
    - Is a LILY_PAD
    - Not occupied by RED/BLUE frogs
    """
    return (0 <= coord.r <= 7 and 
            0 <= coord.c <= 7 and 
            coord in board and 
            board[coord] == CellState.LILY_PAD and
            board[coord] not in {CellState.RED, CellState.BLUE})

def check_in_board(coord: Coord, dir: Direction) -> bool:
    # Detect if the movement complies with the board regulations
    # It is necessary to move within the boundaries of the board
    return (0 <= coord.r + dir.r <= 7) and (0 <= coord.c + dir.c <= 7)

def Astar(
    current: Coord,
    new_pos: Coord,
    move_action: MoveAction,
    g_score: dict[Coord, int],
    came_from: dict[Coord, tuple[Coord, MoveAction]],
    open_set: list[tuple[int, Coord]],
    heuristic: Callable[[Coord], int]
) -> None:
    """
    Common A* pathfinding update logic shared by step and jump moves.
    Updates path data if a better route is found.
    """
    if new_pos not in g_score or g_score[current] + 1 < g_score[new_pos]:
        came_from[new_pos] = (current, move_action)
        g_score[new_pos] = g_score[current] + 1
        f_score = g_score[new_pos] + heuristic(new_pos)
        heapq.heappush(open_set, (f_score, new_pos))

def handle_step_move(
    current: Coord,
    dir: Direction,
    board: dict[Coord, CellState],
    g_score: dict[Coord, int],
    came_from: dict[Coord, tuple[Coord, MoveAction]],
    open_set: list[tuple[int, Coord]],
    heuristic: Callable[[Coord], int]
) -> None:
    """Handle single-step movement in pathfinding."""
    if not check_in_board(current, dir):
        return
    
    new_pos = current + dir
        
    if is_valid_move(new_pos, board):
        Astar(
            current, new_pos, 
            MoveAction(current, [dir]),
            g_score, came_from, 
            open_set, heuristic
        )
            
def handle_jump_move(
    current: Coord,
    dir: Direction,
    board: dict[Coord, CellState],
    g_score: dict[Coord, int],
    came_from: dict[Coord, tuple[Coord, MoveAction]],
    open_set: list[tuple[int, Coord]],
    heuristic: Callable[[Coord], int]
) -> None:
    """Handle jump movement in pathfinding."""
    if not check_in_board(current, dir):
        return
    
    mid_coord = current + dir
    jump_coord = current + dir * 2
    
    if (mid_coord in board and 
        board[mid_coord] == CellState.BLUE and 
        is_valid_move(jump_coord, board)):
        
        Astar(
            current, jump_coord,
            MoveAction(current, [dir]),
            g_score, came_from,
            open_set, heuristic
        )

def search(
    board: dict[Coord, CellState]
) -> list[MoveAction] | None:
    """
    This is the entry point for your submission. You should modify this
    function to solve the search problem discussed in the Part A specification.
    See `core.py` for information on the types being used here.

    Parameters:
        `board`: a dictionary representing the initial board state, mapping
            coordinates to "player colours". The keys are `Coord` instances,
            and the values are `CellState` instances which can be one of
            `CellState.RED`, `CellState.BLUE`, or `CellState.LILY_PAD`.
    
    Returns:
        A list of "move actions" as MoveAction instances, or `None` if no
        solution is possible.
    """

    # The render_board() function is handy for debugging. It will print out a
    # board state in a human-readable format. If your terminal supports ANSI
    # codes, set the `ansi` flag to True to print a colour-coded version!
    print(render_board(board, ansi=True))

    # Do some impressive AI stuff here to find the solution...
    # ...
    # ... (your solution goes here!)
    # ...

    # Heuristic: min Chebyshev distance to any target
    def heuristic(coord: Coord) -> int:
        return min(max(abs(coord.r-t.r), abs(coord.c-t.c)) for t in end)
    
    # Find start (red frog) and targets (bottom row lily pads)
    start = None
    end = []
    for coord, state in board.items():
        if state == CellState.RED:
            start = coord
        if state == CellState.LILY_PAD and coord.r == 7:
            end.append(coord)

    # Edge cases: no start/target or already at target
    if not start or not end:
        return None
    if start.r == 7:
        return None
    
    # A* setup
    open_set = [(0, start)]  # Priority queue (f_score, coord)
    g_score = {start: 0}  # Cost from start
    came_from = {}  # Path reconstruction
    
    # Possible move directions
    directions = [Direction.Right, Direction.Left, 
                 Direction.Down, Direction.DownLeft, Direction.DownRight]
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        # Check if reached any target
        if current in end:
            # Reconstruct path by backtracking
            path = []
            temp_jumps = []  # Stores consecutive jumps
            
            while current in came_from:
                prev, move = came_from[current]
                
                # Group consecutive jumps
                if abs(current.r - prev.r) == 2 or abs(current.c - prev.c) == 2:
                    temp_jumps.append(move)
                else:
                    if temp_jumps:
                        # Combine jumps into single move
                        combined_move = MoveAction(
                            temp_jumps[-1].coord,
                            [d for m in reversed(temp_jumps) for d in m.directions]
                        )
                        path.append(combined_move)
                        temp_jumps = []
                    path.append(move)
                
                current = prev
            
            # Add remaining jumps if any
            if temp_jumps:
                combined_move = MoveAction(
                    temp_jumps[-1].coord,
                    [d for m in reversed(temp_jumps) for d in m.directions]
                )
                path.append(combined_move)
            
            return path[::-1]  # Reverse to get start->end order
        
        # Explore all possible moves
        for dir in directions:
            handle_step_move(current, dir, board, g_score, came_from, open_set, heuristic)
            handle_jump_move(current, dir, board, g_score, came_from, open_set, heuristic)

    return None  # No path found
            
            

    # Here we're returning "hardcoded" actions as an example of the expected
    # output format. Of course, you should instead return the result of your
    # search algorithm. Remember: if no solution is possible for a given input,
    # return `None` instead of a list.
    # return [
    #     MoveAction(Coord(0, 5), [Direction.Down]),
    #     MoveAction(Coord(1, 5), [Direction.DownLeft]),
    #     MoveAction(Coord(3, 3), [Direction.Left]),
    #     MoveAction(Coord(3, 2), [Direction.Down, Direction.Right]),
    #     MoveAction(Coord(5, 4), [Direction.Down]),
    #     MoveAction(Coord(6, 4), [Direction.Down]),
    # ]

    # python -m search < test-vis1.csv
    # python -m search < test-vis.csv