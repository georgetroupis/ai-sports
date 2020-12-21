'''
    Team Rengoku's submission for the 2020 Coder One AI Sports Challenge.
    This bot is used within a game environment allowing two bots to compete against each other.
    The game environment calls the next_move() function for every tick of the game.
'''

# import useful packages
import time
import random
import numpy as np

class agent:
    def __init__(self):
        # the dictionary key is a tuple of coordinates 
        # the dictionary value is a list of length 2 - bomb turn and a bool to check whether it has been activated by a bomb blast
        self.dbombs_timer = {} 

        # the dictionary key is a tuple of the oreblock coord
        # the value is the hp of the ore block
        self.oreblocks_hp = {}
    
        # used to initialize oreblocks hp and location
        self.init_oreblocks_hp = False

        # used to determine which blocks have been targeted by bombs already
        self.blocks_targeted = []

        # determines which tiles are in the path of a bomb which explodes next tick
        self.most_dangerous_tiles = []

        # stores the last 20 actions of the bot, to determine whether moves are repeating
        self.last_20_actions = []


    def next_move(self, game_state, player_state):
        """ 
        This method is called each time the agent is required to choose an action
        """

        ########################
        ###    VARIABLES     ###
        ########################

        # list of all possible actions to take
        self.actions = ['', 'u', 'd', 'l','r','p']
        # store some information about the environment
        # game map is represented in the form (x,y)
        self.cols = game_state.size[0]
        self.rows = game_state.size[1]
        self.oreblocks = game_state.ore_blocks

        self.game_state = game_state 
        self.player_state = player_state
        self.location = player_state.location

        self.ammo = player_state.ammo
        self.bombs = game_state.bombs

        # get our surrounding tiles
        self.surrounding_tiles = self.get_surrounding_tiles(self.location)

        # get list of empty tiles around us
        self.empty_tiles = self.get_empty_tiles(self.surrounding_tiles)

        # Update dbombs dictionary also updates oreblocks health
        self.get_latest_bomb_timers(game_state, self.bombs)

        ########################
        ###       Agent      ###
        ########################

        # 1800 ticks = 180 seconds = 3 minutes
        # (1 tick = 0.1 seconds)
       
        # stores the opponents location for later use
        op_loc = self.get_opponent_location()
        enemy_reachable = True

        # Enemy reachable is True if their is a path between our agent and the opponent agent
        if game_state.tick_number < 100:
            enemy_reachable = self.check_enemy_reachable(op_loc)
            
        # In the last 100 ticks of the match, we want to use all of our bombs to get points.
        # If their is no path between our agent and the enemy agent, we also need to explode blocks to create a path 
        if 1800 - game_state.tick_number < 100 or (enemy_reachable is False and game_state.tick_number < 100):

            # Check to see if we are standing on a bomb
            if self.location in self.bombs:
                action = self.move_off_bomb()

            # check to see if the agent is on a dangerous tile with one HP
            elif self.location in self.most_dangerous_tiles and player_state.hp == 1:
               action = self.move_off_dangerous_tiles()

            # if the opponent is next to us and the opponent is trapped, place a bomb 
            elif op_loc in self.surrounding_tiles and self.isTrap(op_loc):
                action = self.place_bomb(["sb", "ob"])

            # if we have ammo to use 
            elif player_state.ammo>0:
                
                # creates arrays storing the possible bomb locations that will explode three, two and one block(s) respectively
                three_block_pos, two_block_pos, one_block_pos = self.detect_bomb_placements(["sb"])			

                # Attempts to find a path to each bomb placement and returns an action to get to that location, or place a bomb if already on the location
                result = self.get_optimal_bomb_placement(three_block_pos, two_block_pos, one_block_pos)
                if result is not False:
                    action = result

                # No path to any bombs, find ammo 
                else:
                    position = self.find_closest(game_state.ammo)
                    if position is not False:
                        action = self.move_to_tile(position)

                    # No ammo, we can just stand still
                    else:
                        action = ''
            
            # if we have no ammo, try and find treasure 
            elif len(game_state.treasure) >0:
                
                position = self.find_closest(game_state.treasure)
                if position is not False:
                    action = self.move_to_tile(position)
                else:

                    position = self.find_closest(game_state.ammo)
                    if position is not False:
                        action = self.move_to_tile(position)
                    else:
                        action = ''

            # if there is ammo on the map, pick it up
            elif len(game_state.ammo) > 0:
                position = self.find_closest(game_state.ammo)

                if position is not False:
                    action = self.move_to_tile(position)
                
                # Nothing else to do, go to an optimal location on the map
                else:
                    action = self.get_optimal_positioning()
            
            else:
                action = self.get_optimal_positioning()

        # not in the last couple of seconds, execute as normally 
        else:

            # Check to see if we are standing on a bomb
            if self.location in self.bombs:
                action = self.move_off_bomb()

            # check to see if the agent is on a dangerous tile with one HP
            elif self.location in self.most_dangerous_tiles:
                action = self.move_off_dangerous_tiles()

            # if the opponent is next to us and the opponent is trapped, place a bomb 
            elif op_loc in self.surrounding_tiles and self.isTrap(op_loc):
                action = self.place_bomb(["sb", "ob"]) 

            else: 

                # find ammo on map
                position = self.find_closest(game_state.ammo)
                if position is not False:
                    action = self.move_to_tile(position)
                else: 
                    position = self.find_closest(game_state.treasure)
                    if position is not False:
                        action = self.move_to_tile(position)

                    # go to centre of map 
                    else:
                        action = self.get_optimal_positioning()
        
        # get the updated dangerous tiles for the next tick
        self.most_dangerous_tiles = self.get_most_dangerous_tiles(game_state)
        
        # Stop gap measure to prevent repetitive movements
        action = self.break_action_loop(action)
        return action

    ########################
    ###     HELPERS      ###
    ########################

    '''
        From a list of bombs return a dict of bombs 
        the dictionary key is a tuple of coordinates 
        the dictionary value is a list of length 2 - bomb turn and a bool to check whether it has been activated by a bomb blast
    '''
    def get_latest_bomb_timers(self, game_state, bombs):
        # a dictionary copy is needed as dictionary size is not allowed to change during iteration
        d_copy = self.dbombs_timer.copy()
        before_last_tick = 33
        bomb_detonate_list = []
        other_bombs = []
        
        # Delete old bomb positions if it doesn't exist anymore in the bombs list
        for bomb, value in d_copy.items():
            if bomb not in bombs:
                #("dBomb coordinate {0} with value {1} has exploded".format(bomb,value))
                self.update_oreblock_health(game_state,bomb)
                del self.dbombs_timer[bomb]

        # Keep track of new bombs in the dictionary and their respective timer
        for bomb in bombs:
            if bomb not in d_copy:
                # Enter a new bomb in to the dictionary
                self.dbombs_timer[bomb] = [0,False]
            else:
                # Check if the bomb is going to explode in the next turn
                if self.dbombs_timer[bomb][0]  >= before_last_tick or self.dbombs_timer[bomb][1] == True:
                    bomb_detonate_list.append(bomb)
            self.dbombs_timer[bomb][0] += 1
            # if self.dbombs_timer[bomb][0] == 35:
            #     del self.dbombs_timer[bomb]

        # Check if any of the bomb's blast causes other bombs to activate
        for bomb in bombs:
            if bomb not in bomb_detonate_list:
                other_bombs.append(bomb)
        blast_range_list = self.get_blast_range_list(game_state, bomb_detonate_list,exclude_entity='b')

        for bomb in other_bombs:
            if bomb in blast_range_list:
                # #("Bomb at {0} has been detected to be at a blast range".format(bomb))
                self.dbombs_timer[bomb][1] = True
                    
        return self.dbombs_timer

    '''
        from a dictionary of bombs timers, return a list of tiles that are going to explode in the next 2 ticks
    '''
    def get_most_dangerous_tiles(self, game_state):

        most_dangerous_tiles = []
        dangerous_bomb_list = []
        before_last_tick = 32

        for key, value in self.dbombs_timer.items():
            # Check if its going to explode in the next tick due to tick 35 or if it has been activated
            if value[0] >= before_last_tick or value[1] == True:
                dangerous_bomb_list.append(key)
        
        most_dangerous_tiles = self.get_blast_range_list(game_state,dangerous_bomb_list,exclude_entity='b')

        ##(dbombs_timer)
        ##(most_dangerous_tiles)

        return most_dangerous_tiles

    '''
        from a list of bombs, return a list of empty tiles that is in range of an explosion
    '''
    def get_blast_range_list(self, game_state, bombs, exclude_entity = None):
 
        dangerous_tiles = []
        block_tiles = ['ib','sb','ob','b'] 
        if exclude_entity:
            block_tiles.remove(exclude_entity)

        # bombs is a list of tuples
        for tuples in bombs:
        # tiles in the bomb range
            center = tuples
            dangerous_tiles.append(center)

            tile_up = (tuples[0], tuples[1] + 1)
            # Check for validity and any blockages
            if game_state.is_in_bounds(tile_up) and (game_state.entity_at(tile_up) not in block_tiles):
                dangerous_tiles.append(tile_up)
                # next check if there is any blockages above it.
                tile_up_up = (tuples[0], tuples[1]+2)
                if game_state.is_in_bounds(tile_up_up) and (game_state.entity_at(tile_up_up) not in block_tiles):
                    dangerous_tiles.append(tile_up_up)

            tile_down = (tuples[0], tuples[1] - 1)
            if game_state.is_in_bounds(tile_down) and (game_state.entity_at(tile_down) not in block_tiles):
                dangerous_tiles.append(tile_down)
                tile_down_down = (tuples[0], tuples[1] - 2)
                if game_state.is_in_bounds(tile_down_down) and (game_state.entity_at(tile_down_down) not in block_tiles):
                    dangerous_tiles.append(tile_down_down)

            tile_left = (tuples[0]-1, tuples[1])
            if game_state.is_in_bounds(tile_left) and (game_state.entity_at(tile_left) not in block_tiles):
                dangerous_tiles.append(tile_left)
                tile_left_left = (tuples[0]-2, tuples[1])
                if game_state.is_in_bounds(tile_left_left) and (game_state.entity_at(tile_left_left) not in block_tiles):
                    dangerous_tiles.append(tile_left_left)	

            tile_right = (tuples[0] + 1, tuples[1])
            if game_state.is_in_bounds(tile_right) and (game_state.entity_at(tile_right) not in block_tiles):
                dangerous_tiles.append(tile_right)
                tile_right_right = (tuples[0] + 2, tuples[1])
                if game_state.is_in_bounds(tile_right_right) and (game_state.entity_at(tile_right_right) not in block_tiles):
                    dangerous_tiles.append(tile_right_right)

            # remove duplicates
            dangerous_tiles = list(dict.fromkeys(dangerous_tiles))

        return dangerous_tiles

    '''
        returns a list of tiles that are empty
    '''
    def get_empty_tiles(self, tiles):

        # empty list to store our empty tiles
        empty_tiles = []

        for tile in tiles:
            if not self.game_state.is_occupied(tile):
                # the tile isn't occupied, so we'll add it to the list
                empty_tiles.append(tile)

        return empty_tiles


    '''
        returns a list of tiles that surround the location given
    '''
    def get_surrounding_tiles(self, location):

        # find all the surrounding tiles relative to us
        # location[0] = col index; location[1] = row index
        tile_up = (location[0], location[1]+1)	
        tile_down = (location[0], location[1]-1)     
        tile_left = (location[0]-1, location[1]) 
        tile_right = (location[0]+1, location[1]) 		 

        # combine these into a list
        all_surrounding_tiles = [tile_up, tile_down, tile_left, tile_right]

        # we'll need to remove tiles that cross the border of the map
        # start with an empty list to store our valid surrounding tiles
        valid_surrounding_tiles = []

        # loop through our tiles
        for tile in all_surrounding_tiles:
            # check if the tile is within the boundaries of the game
            if self.game_state.is_in_bounds(tile):
                # if yes, then add them to our list
                valid_surrounding_tiles.append(tile)

        return valid_surrounding_tiles

    '''
        given a tile determine if it is a trap (ie, is no empty tile to go to after moving there)
    '''
    def isTrap(self, tile_pos):

        # get the surrounding tiles of that area
        surrounding_tiles = self.get_surrounding_tiles(tile_pos)
        # get all the empty tiles
        empty_tiles = self.get_empty_tiles(surrounding_tiles)
        # is there any empty tiles
        if empty_tiles:
            return False 
        # there is no empty tile that
        else:
            return True 


    '''
        returns the direction to move in, to go to the desired tile from the current location
    '''
    def move_to_tile(self, tile):

        actions = ['', 'u', 'd', 'l','r','p']

        # see where the tile is relative to our current location
        diff = tuple(x-y for x, y in zip(self.location, tile))

        if tile in self.most_dangerous_tiles and self.location not in self.most_dangerous_tiles:

            #("DONT WANT TO MOVE INTO DANGER")
            return ""

        # return the action that moves in the direction of the tile
        if diff == (0,1):
            action = 'd'
        elif diff == (1,0):
            action = 'l'
        elif diff == (0,-1):
            action = 'u'
        elif diff == (-1,0):
            action = 'r'
        else:
            action = ''
        return action
    
    '''
        from a list of oreblocks check if there are any bomb that has  exploded near it
        update the orebock dictionary
    '''
    def update_oreblock_health(self,game_state,bomb):
        # Get latest ore blocks
        oreblocks = game_state.ore_blocks

        # TAKE NOTE IF TWO ORE BLOCKS SPAWN SIDE BY SIDE A BOMB THAT EXPLODE 
        blast_range = self.get_blast_range_list(game_state,[bomb],exclude_entity='ob')

        for oreblock in oreblocks:
            # check if it still exists in the oreblock_hp dict
            if oreblock in self.oreblocks_hp:
                if oreblock in blast_range:
                    self.oreblocks_hp[oreblock] -= 1
                    if self.oreblocks_hp[oreblock] == 0:
                        del self.oreblocks_hp[oreblock]

        return  

    '''
        Takes a list of objects, and returns the distance to 
    '''
    def find_closest(self, object_list):
        distances = []
        for pos in object_list:
            
            res = self.dij(self.location, pos)
            if res is not False:
                distances.append(res)

        if len(distances) == 0:
            return False
        # sort the distances by the minimum to maximum 
        sorted(distances, key = lambda x: x[1])

        return distances[0][0]

    '''
        returns tuple of next place to move and distance to the goal
        return value is False if square is not able to be moved onto
    '''
    def dij(self, start_pos, goal_pos):
        start_pos = self.graph_pos_to_np_pos(start_pos)
        goal_pos = self.graph_pos_to_np_pos(goal_pos)

        # #("Cur Location is {0}: goal pos is {1}".format(start_pos, goal_pos))

        opponent_id = abs(1- self.player_state.id)
        ''' 
            3: can't walk if block there
            4: can't walk if there is a hole
        '''
        ILLEGAL_BLOCKS = (opponent_id, "ib", "sb", "ob")


        rows = self.game_state.size[1]
        cols = self.game_state.size[0]
        
        if start_pos == goal_pos:
            #("on goal position")
            return ((-1, -1), 0)

        # queue to insert positions into 
        queue = [] 
        found = False

        # VISITED ARRAY
        visited = np.full((rows, cols), False)
        visited[start_pos] = True

        # distance to the location
        dist = np.full((rows, cols), 2^32 -1)
        dist[start_pos] = 0

        # this was the previous location
        predecessor = np.full((rows, cols), -10, dtype=object)
        predecessor[start_pos] = -1

        queue.append(start_pos)

        count = 0
        # while there are unvisited nodes in the queue 
        while len(queue):


            if found:
                break

            # Pops a vertex with the smallest distance 
            cur_pos = queue.pop(0)

            # #("Popped position is {0}".format(cur_pos))
            surround_tiles = self.get_surrounding_tiles_np(cur_pos)

            # #(surround_tiles)

            for pos in surround_tiles:
                # if tile is not walkable - ie. is a hole, block or other player OR has been visited OR is occupied by a player on the first iteration
                if self.entity_at_np_pos(pos) in ILLEGAL_BLOCKS or visited[cur_pos] is True or (self.entity_at_np_pos(pos) == opponent_id and count == 0): 
                    continue
                
                # check if distance to this block is the smallest distance we have seen so far
                if dist[cur_pos] + 1 < dist[pos]:
                    
                    #update shortest distance
                    dist[pos] = dist[cur_pos] +1
                    predecessor[pos] = cur_pos
                    queue.append(pos)
                
                #if the position is the goal_position
                if pos == goal_pos:
                    found = True
                    break

            # set the current position to being visited
            visited[cur_pos] = True
            count+=1

        # if a path was found
        if found:
            ret_pos = goal_pos

            # loop through predecessors until the iteration before the start_pos
            while predecessor[predecessor[ret_pos]] != -1:
                ret_pos = predecessor[ret_pos]
            
            ret_pos = self.np_pos_to_graph_pos(ret_pos)
            return (ret_pos, dist[goal_pos])

        # if no path return False
        else:
            return False

    '''
        converts a tile location from an numpy index to an (x, y) index
    '''
    def np_pos_to_graph_pos(self, np_pos):
        rows = self.game_state.size[1]

        graph_x = np_pos[1]
        graph_y = abs(rows - np_pos[0] -1)

        return (graph_x, graph_y)

    '''
        returns the entity at the given numpy index
    '''
    def entity_at_np_pos(self, np_pos):
        graph_pos = self.np_pos_to_graph_pos(np_pos)
        return self.game_state.entity_at(graph_pos)

    '''
        converts a tile location from an (x, y) index to a numpy index
    '''
    def graph_pos_to_np_pos(self, graph_pos):
        
        rows = self.game_state.size[1]

        graph_x = graph_pos[0]
        graph_y = graph_pos[1]

        np_col = graph_x
        np_row = abs(graph_y- rows +1)

        return (np_row, np_col)

    '''
    takes an np_tile and returns an np_tile
    '''
    def get_surrounding_tiles_np(self, np_loc):
        
        location = self.np_pos_to_graph_pos(np_loc)

        # find all the surrounding tiles relative to us
        # location[0] = col index; location[1] = row index
        tile_up = (location[0], location[1]+1)	
        tile_down = (location[0], location[1]-1)     
        tile_left = (location[0]-1, location[1]) 
        tile_right = (location[0]+1, location[1])


        # combine these into a list
        all_surrounding_tiles = [tile_up, tile_down, tile_left, tile_right]

        # we'll need to remove tiles that cross the border of the map
        # start with an empty list to store our valid surrounding tiles
        valid_surrounding_tiles = []

        # loop through our tiles
        for tile in all_surrounding_tiles:
            # check if the tile is within the boundaries of the game
            if self.game_state.is_in_bounds(tile):
                # if yes, then add them to our list

                np_tile = self.graph_pos_to_np_pos(tile)
                valid_surrounding_tiles.append(np_tile)

        return valid_surrounding_tiles

    '''
        Returns the blocks in the bomb radius 
    '''
    def blocks_in_bomb_radius(self, location, block_list):
        
        # find all the surrounding tiles relative to us
        # location[0] = col index; location[1] = row index
        tile_up = (location[0], location[1]+1)	
        tile_down = (location[0], location[1]-1)     
        tile_left = (location[0]-1, location[1]) 
        tile_right = (location[0]+1, location[1])

        tile_up_up = (location[0], location[1]+2)	
        tile_down_down = (location[0], location[1]-2)     
        tile_left_left = (location[0]-2, location[1]) 
        tile_right_right = (location[0]+2, location[1])
        
        # combine these into a list
        tiles_up = [tile_up, tile_up_up]
        tiles_down = [tile_down, tile_down_down]
        tiles_left = [tile_left, tile_left_left]
        tiles_right = [tile_right, tile_right_right]
        
        stop_blast = ["b", "ib"]

        blocks_in_radius = []

        # we'll need to remove tiles that cross the border of the map
        # start with an empty list to store our valid surrounding tiles

        # loop through our tiles
        for tile in tiles_up:
            # check if the tile is a bomb, if so, add it to the blocks_in_radius list
            if self.game_state.entity_at(tile) in block_list:
                if tile not in self.blocks_targeted:
                    blocks_in_radius.append(tile)
                break
            elif self.game_state.entity_at(tile) in stop_blast:
                break

        
        for tile in tiles_down:
            
            if self.game_state.entity_at(tile) in block_list:
                
                if tile not in self.blocks_targeted:
                    blocks_in_radius.append(tile)

                break
            elif self.game_state.entity_at(tile) in stop_blast:
                break

        for tile in tiles_left:
            if self.game_state.entity_at(tile) in block_list:
                if tile not in self.blocks_targeted:
                    blocks_in_radius.append(tile)
                break
            elif self.game_state.entity_at(tile) in stop_blast:
                break
        for tile in tiles_right:
            if self.game_state.entity_at(tile) in block_list:
                if tile not in self.blocks_targeted:
                    blocks_in_radius.append(tile)
                break
            elif self.game_state.entity_at(tile) in stop_blast:
                break
    
        return blocks_in_radius

    '''
        Returns a list of pathable tiles in the centre of the map
    '''
    def find_centre_map(self):

        # 5, 6 is centre for x 
        # 4 or 5 is centre for y

        opponent_id = self.game_state.opponents(self.player_state.id)[0]

        ILLEGAL_BLOCKS = (opponent_id, "ib", "sb", "ob")


        centre_tiles = [(5, 4), (5 ,5), (6, 4), (6, 5)]
        safe_centre_tiles = []

        perimeter_tiles = [(4, 4), (4, 5), (5, 6), (6, 6), (7, 4), (7, 5), (5, 3), (6, 3)]

        while len(safe_centre_tiles) == 0:
            for tile in centre_tiles:
                if self.game_state.entity_at(tile) not in ILLEGAL_BLOCKS:
                    safe_centre_tiles.append(tile)
            centre_tiles = perimeter_tiles
        return safe_centre_tiles

    '''
        returns bomb placements that would explode two bombs and 
        placements that would explode three bombs
        Only returns the position if it is empty OR is inhabitant by THIS player 
        (will not return if inhabited by the opposition player)
    '''
    def detect_bomb_placements(self, block_list):

        two_block_pos = []
        three_block_pos = []
        one_block_pos = []
        
        rows = self.game_state.size[1]
        cols = self.game_state.size[0]
        
        walkable_blocks = ["t", "a", self.player_state.id, None]

        not_occupied = []

        # iterates through every position on the board
        for x in range(cols):
            for y in range(rows):

                # if the position is empty or inhabited by this player
                if self.game_state.entity_at((x,y)) in walkable_blocks:
                    
                    # count the number of blocks in the two tile blast radius 
                    count = len(self.blocks_in_bomb_radius((x,y), block_list))

                    if count == 3:
                        three_block_pos.append((x, y))
                    elif count == 2:
                        two_block_pos.append((x, y))
                    elif count == 1:
                        one_block_pos.append((x, y))			
                    
        return three_block_pos, two_block_pos, one_block_pos

    '''
        checks whether there is a path to that location. If there is a path to at least one bomb placement, 
            finds the shortest distance to a bomb. Returns an instruction to either place a bomb (if at a good location),
            or an instruction to move to the next best tile.
    '''
    def check_viability_of_placement(self, block_array):
        if len(block_array) == 0:
            return False
        distances = []

        # finds the shortest paths for each position in 'block_array'
        for pos in block_array:

            #call dijsktra for the path
            res = self.dij(self.location, pos)

            # if Dijkstra finds a path, append the value to the distance array
            if res is not False:
                distances.append(res)

                # if we are on a good block, no need to compute the rest
                if(res[1] == 0):
                    break
        # Return false if there are no paths to any of the block placements
        if len(distances) == 0:
            return False

        # sort the distances by the minimum to maximum 
        sort_dist = sorted(distances, key = lambda x: x[1])
        
        # if the distance to the optimal location is 0, we should place a bomb here
        if sort_dist[0][1] == 0:
            return "ON_BEST"
            
        # otherwise move to the next best tile
        else:
            return sort_dist[0][0]

    '''
        adds list of blocks which are targeted by a bomb to self.blocks_targeted. 
        Then returns the action to place a bomb
    '''
    def place_bomb(self, block_list):

        ret_targeted = self.blocks_in_bomb_radius(self.location, block_list)
        for block in ret_targeted:
            self.blocks_targeted.append(block)
        #("placing bomb")
        return "p"

    '''
        returns location of opponent
    '''
    def get_opponent_location(self):

        opponent_id = abs(1- self.player_state.id)
        rows = self.game_state.size[1]
        cols = self.game_state.size[0]

        # iterates through every position on the board
        for x in range(cols):
            for y in range(rows):
                if self.game_state.entity_at((x, y)) == opponent_id:
                    return (x, y)

        return False

    '''
        returns the tile that is next to the opponent but on the centre side 
    '''
    def goal_side_of_opponent(self):
        opponent_location = self.get_opponent_location()

        opponent_id = abs(1- self.player_state.id)


        if opponent_location is False:
            return False
        left = False
        down = False
        ILLEGAL_BLOCKS = (opponent_id, "ib", "sb", "ob")
        
        # work out if they are in left half or right half
        if opponent_location[0] <= 5:
            # in the right half 
            left = True

        if opponent_location[1] <= 4:
            down = True

        tiles = self.get_surrounding_tiles(opponent_location)
        legit_tiles = []
        
        for tile in tiles:
            if self.game_state.entity_at(tile) not in ILLEGAL_BLOCKS:
                legit_tiles.append(tile)

        if left is True:
            if (opponent_location[0] +1, opponent_location[1]) in legit_tiles:
                return (opponent_location[0] +1, opponent_location[1])
        else:
            if (opponent_location[0] -1, opponent_location[1]) in legit_tiles:
                return (opponent_location[0] -1, opponent_location[1]) 

        if down is True:
            if (opponent_location[0], opponent_location[1] +1) in legit_tiles:
                return (opponent_location[0], opponent_location[1]+1)
        
        else: 
            if (opponent_location[0], opponent_location[1] -1) in legit_tiles:
                return (opponent_location[0], opponent_location[1]-1)
            
        return False

    '''
        This is a rough fix. It checks the last 20 actions. If the last 20 actions consists of ANY 6 similar moves except for idling
        Break execute get a random move to refresh dij  
    '''

    def break_action_loop(self, action):
        self.last_20_actions.append(action)

        # Refresh last 20 actions
        if len(self.last_20_actions) > 20:
            self.last_20_actions = []
            return action

        if len(self.last_20_actions) <= 20:
            counts = {}
            for i in self.last_20_actions:
                if i != '':
                    counts[i] = counts.get(i, 0)+1
            for i in counts:
                if counts[i]>9:

                    self.last_20_actions = []
                    return random.choice(['u', 'd', 'l','r'])
            return action
        else:
            return action

    '''
        Takes the opponents location as input. 
        Checks whether their is a path from our character to the opoonent.
    '''
    def check_enemy_reachable(self, op_loc):
        if op_loc is False:
            return False
        else:
            tiles_sur_opponent = self.get_surrounding_tiles(op_loc)

            for tile in tiles_sur_opponent:
                path_to_op = self.dij(self.location, tile)
                if path_to_op is not False:
                    return True
        return False

    '''
        Returns an action to remove the agent off of a bomb
    '''
    def move_off_bomb(self):
        # find a safe  tile from surrounding tile 
        safe_tiles = []
        for tile in self.empty_tiles:
            if tile not in self.bombs and not self.isTrap(tile):
                safe_tiles.append(tile)
        if safe_tiles:

            #(safe_tiles)
            action = self.move_to_tile(random.choice(safe_tiles))

        elif self.location in self.most_dangerous_tiles:
            action = self.move_to_tile(empty_tiles[0])
        else:
            # No safe tile just move randomly RIP
            action = random.choice(self.actions) 

        return action

    '''
        Returns an action to remove the agent off of a dangerous tile
    '''
    def move_off_dangerous_tiles(self):
        # Am i anywhere near a bomb blast range in 2 turns? If i am get out of there
        # find a safe  tile from surrounding tile 
        safe_tiles = []
        for tile in self.empty_tiles:
            if tile not in self.most_dangerous_tiles:
                safe_tiles.append(tile)
        if safe_tiles:
            #("MOVING AWAY")
            #(safe_tiles)
            action = self.move_to_tile(random.choice(safe_tiles))
        elif self.location in self.most_dangerous_tiles:
            action = self.move_to_tile(empty_tiles[0])
        else:
            # No safe tile just move randomly RIP
            action = random.choice(self.actions) 

        return action


    '''
        Uses Dijkstra's algorithm to detect a bomb position which will: 
            1) explode the most amount of blocks possible
            2) if there are more than one of these positions, finds the closest one of these to the agents current location
        If there are no possible positions to explode blocks, returns False
    '''
    def get_optimal_bomb_placement(self, three_block_pos, two_block_pos, one_block_pos):
        # #("------------three block pos-----------------", three_block_pos)
        # if there are tiles with 3 blocks surrounding, check to see if they are 
        # able to be moved to 

        res_check = self.check_viability_of_placement(three_block_pos)
        if res_check == "ON_BEST":
            action = self.place_bomb(["sb", "ob"])
        elif res_check is not False:
            action = self.move_to_tile(res_check)								
        else:
            # #("------------------two block pos----------------", two_block_pos)
            res_check = self.check_viability_of_placement(two_block_pos)

            if res_check == "ON_BEST":
                action = self.place_bomb(["sb", "ob"])
            elif res_check is not False:
                action = self.move_to_tile(res_check)
            else:
                #print("----------------one block pos-----------------", one_block_pos)
                res_check = self.check_viability_of_placement(one_block_pos)

                if res_check == "ON_BEST":
                    action = self.place_bomb(["sb", "ob"])
                elif res_check is not False:
                    action = self.move_to_tile(res_check)

                else:
                    return False

        return action 

    '''
        Returns the action to get to the 'optimal' position on the map.
        The optimal position is a tile that is directly next to the opponent but one tile closer to the centre.
        If this tile is unavailable, it is a tile in the centre of the map. 
    '''
    def get_optimal_positioning(self):

        position = self.goal_side_of_opponent()
        if position is not False:
            next_tile = self.dij(self.location, position)

            if next_tile is not False:
                action = self.move_to_tile(next_tile[0])
            else:
                safe_centre_tiles = self.find_centre_map()
                position = self.find_closest(safe_centre_tiles)
                if position is not False:
                    action = self.move_to_tile(position)
                else:
                    action = ""
        else:
            safe_centre_tiles = self.find_centre_map()
            position = self.find_closest(safe_centre_tiles)
            if position is not False:
                action = self.move_to_tile(position)
            else:
                action = ""

        return action
