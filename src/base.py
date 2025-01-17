
class Boxtype:
    id: int
    l: int; w: int; h: int
    rot_l: bool; rot_w: bool; rot_h: bool
    weight: int
    stacking_weight_resistance: int
    volume: int

    def __init__(self, id, l, w, h, rot_l=True, rot_w=True, rot_h=True, weight=1, stacking_weight_resistance=1000):
        self.id = id
        self.l = l; self.w = w; self.h = h
        self.rot_l, self.rot_w, self.rot_h = rot_l, rot_w, rot_h
        self.weight = weight
        self.stacking_weight_resistance= stacking_weight_resistance
        self.volume = l*w*h

# Items are pairs (Boxtype, quantity)
class Itemdict(dict):
    def __iadd__(self, other):
        for key in other:
            if key in self:
                self[key] += other[key]
            else:
                self[key] = other[key]
        return self

    def __isub__(self, other):
        for key in other:
            if key in self:
                self[key] -= other[key]
            else:
                self[key] = -other[key]
        return self
  
    def __le__(self, other):
        for key in other:
            if self[key] > other[key]:
                return False
        return True
            
    def __copy__(self):
        return Itemdict(self)

#An Aabb is cuboid+location
#Useful for representing free space cuboids and placed blocks
class Aabb:
    xmin: int; xmax: int
    ymin: int; ymax: int
    zmin: int; zmax: int
    l: int; w: int; h: int
    manhattan: int
    volume: int
    weight: int
    stacking_weight_resistance: int
    covered_surface: int
    covered_surface_face: dict()
    

    def __init__(self, xmin, xmax, ymin, ymax, zmin, zmax):
        self.xmin = xmin; self.xmax = xmax
        self.ymin = ymin; self.ymax = ymax
        self.zmin = zmin; self.zmax = zmax
        self.volume = (xmax-xmin)*(ymax-ymin)*(zmax-zmin)
        self.l = xmax-xmin; self.w = ymax-ymin; self.h = zmax-zmin
        self.manhattan = self.xmin + self.ymin + self.zmin
        self.covered_surface = 0
        self.covered_surface_face = {'X1':0,'X2':0,'Y1':0,'Y2':0}
        self.weight = 0
        self.stacking_weight_resistance = 0
    
    # returns true if aabb is inside self
    def strict_intersects(self, aabb):
        return self.xmin < aabb.xmax and self.xmax > aabb.xmin and self.ymin < aabb.ymax and self.ymax > aabb.ymin and self.zmin < aabb.zmax and self.zmax > aabb.zmin
    
    # returns true if aabb intersects self
    def intersects(self, aabb):
        return self.xmin <= aabb.xmax and self.xmax >= aabb.xmin and self.ymin <= aabb.ymax and self.ymax >= aabb.ymin and self.zmin <= aabb.zmax and self.zmax >= aabb.zmin
    
    # returns a list of aabbs that are the result of substracting aabb from self    
    def subtract(self, aabb):
        sub = list()
        if aabb.xmax < self.xmax:
            sub.append(Aabb(aabb.xmax, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax))
        if aabb.ymax < self.ymax:
            sub.append(Aabb(self.xmin, self.xmax, aabb.ymax, self.ymax, self.zmin, self.zmax))
        if aabb.zmax < self.zmax:
            sub.append(Aabb(self.xmin, self.xmax, self.ymin, self.ymax, aabb.zmax, self.zmax))
        if aabb.xmin > self.xmin:
            sub.append(Aabb(self.xmin, aabb.xmin, self.ymin, self.ymax, self.zmin, self.zmax))
        if aabb.ymin > self.ymin:
            sub.append(Aabb(self.xmin, self.xmax, self.ymin, aabb.ymin, self.zmin, self.zmax))
        if aabb.zmin > self.zmin:
            sub.append(Aabb(self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, aabb.zmin))
        return sub
    
    def can_contain(self, aabb):
        return self.l >= aabb.l and self.w >= aabb.w and self.h >= aabb.h
    
    # aabb is contained in self
    def __ge__(self, aabb):
        return self.xmin <= aabb.xmin and self.xmax >= aabb.xmax and self.ymin <= aabb.ymin and self.ymax >= aabb.ymax and self.zmin <= aabb.zmin and self.zmax >= aabb.zmax
    
    def __str__(self):
        return "Aabb: xmin: " + str(self.xmin) + " xmax: " + str(self.xmax) + " ymin: " + str(self.ymin) + " ymax: " + str(self.ymax) + " zmin: " + str(self.zmin) + " zmax: " + str(self.zmax) + " weight: " + str(self.weight) + " stacking weight resistance: " + str(self.stacking_weight_resistance)


# # Items are pairs (Boxtype, quantity)
class Itemdict(dict):
  def __iadd__(self, other):
      for key in other:
          if key in self:
              self[key] += other[key]
          else:
              self[key] = other[key]
      return self

  def __isub__(self, other):
      for key in other:
          if key in self:
              self[key] -= other[key]
          else:
              self[key] = -other[key]
      return self


class Space(Aabb):
    manhattan: int #the manhattan distance to the closest corner of the space to a block's corner
    corner_point: list() #the closest corner of the space to a block's corner

    #static variable
    filling = "bottom-up" #taba origin 000000 the filling method used by the algorithm
    vertical_stability = False#True # boxes must be completly supported

    def __init__(self, xmin, xmax, ymin, ymax, zmin, zmax, block):
        super().__init__(xmin, xmax, ymin, ymax, zmin, zmax)
        self.container_block = block
        self.corner_point = [xmin, ymin, zmin]
        xdist = xmin; ydist = ymin; zdist = zmin
        if Space.filling == "bottom-up": zdist = 1000*zmin

        #compute manhattan distance to the closest corner of the block
        if block.l-xmax < xmin and Space.filling != "origin": #aqui habia origin
            xdist = block.l-xmax
            self.corner_point[0] = xmax

        if block.w-ymax < ymin and Space.filling != "origin": #aqui habia origin
            ydist = block.w-ymax
            self.corner_point[1] = ymax

        if block.h-zmax < zmin and Space.filling == "free": 
            zdist = block.h-zmax
            self.corner_point[2] = zmax

        self.manhattan = xdist + ydist + zdist
        
    # returns a list of aabbs that are the result of substracting aabb from self    
    def subtract(self, aabb, container_block):
        sub = list()
        if aabb.xmax < self.xmax:
            sub.append(Space(aabb.xmax, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax, container_block))
        if aabb.ymax < self.ymax:
            sub.append(Space(self.xmin, self.xmax, aabb.ymax, self.ymax, self.zmin, self.zmax, container_block))
        if aabb.zmax < self.zmax:
          if Space.vertical_stability==False:
            sub.append(Space(self.xmin, self.xmax, self.ymin, self.ymax, aabb.zmax, self.zmax, container_block))
          else:
            sub.append(Space(aabb.xmin, aabb.xmax, aabb.ymin, aabb.ymax, aabb.zmax, self.zmax, container_block))
        if aabb.xmin > self.xmin:
            sub.append(Space(self.xmin, aabb.xmin, self.ymin, self.ymax, self.zmin, self.zmax, container_block))
        if aabb.ymin > self.ymin:
            sub.append(Space(self.xmin, self.xmax, self.ymin, aabb.ymin, self.zmin, self.zmax, container_block))
        if aabb.zmin > self.zmin:
            sub.append(Space(self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, aabb.zmin, container_block))
        return sub

    


    
#FreeSpaace represent the free space inside a block
#Consists of a list of free space aabbs (spaces)
class FreeSpace:
    spaces : list() # list of aabbs
    def __init__(self, aabb=None):
        self.spaces = list()
        if aabb is not None:
          self.spaces.append(aabb)

    def remove_nonmaximal_spaces(self, aabbs):
      #sort aabbs by volume
      aabbs.sort(key=lambda aabb: aabb.volume, reverse=True)

      for i in range(len(aabbs)):
        if i>=len(aabbs): break
        for j in range(i+1, len(aabbs)):   
          if j>=len(aabbs): break
          if aabbs[i] >= aabbs[j]:
              aabbs.remove(aabbs[j])
              j -= 1
          
    def crop(self, aabb, container_block):
        new_spaces = list()
        to_remove = list()
        for space in self.spaces:
            if space.intersects(aabb):                
                if space.strict_intersects(aabb):
                    sub = space.subtract(aabb, container_block)
                    for s in sub: new_spaces.append(s)
                else:
                    new_spaces.append(space)
                
                to_remove.append(space)

        for sp in to_remove: self.spaces.remove(sp)

        self.remove_nonmaximal_spaces(new_spaces)

        self.spaces.extend(new_spaces)

    #compute the free space closer to the origin (manhattan distance)
    def closest_space(self):
        min = 1000000; cspace=None
        for space in self.spaces:
            if space.manhattan < min: 
              min = space.manhattan
              cspace = space
        if cspace == None: return None
        else: return cspace

    #remove all spaces that cannot be filled by a boxtype
    def filter(self, items):
        to_remove = list()
        for space in self.spaces:
            remove = True
            for item in items:
                if items[item]>0 and space.l >= item.l and space.w >= item.w and space.h >= item.h: 
                    remove = False
                    break
            if remove==True: to_remove.append(space)

        for sp in to_remove: self.spaces.remove(sp)

    def __str__(self):
        _str = ""
        for space in self.spaces:
          _str+= str(space) +"\n"
        return _str

from copy import copy

#A block is compund by a set of items(boxtype+quantity)
#The container is a block
class Block:
    l: int; w: int; h: int
    occupied_volume: int
    weight: int
    stacking_weight_resistance: int
    volume: int
    items: Itemdict() # Boxtype: int
    free_space: FreeSpace() # list of free spaces
    aabbs: list() # placed blocks
    tokens: list()

     # copy blocks and items
    def __copy__(self):
        return Block(copy_block=self) 

    def __init__(self, boxtype=None, rot=True, l=0, w=0, h=0, weight=1, stacking_weight_resistance=2, copy_block=None):

        # print(f'w: {weight}   swr: {stacking_weight_resistance}')

        if copy_block is not None:
            # copy blocks and items
            self.l = copy_block.l
            self.w = copy_block.w
            self.h = copy_block.h
            self.weight = copy_block.weight
            self.stacking_weight_resistance = copy_block.stacking_weight_resistance
            self.occupied_volume = copy_block.occupied_volume
            self.volume = copy_block.volume
            self.items = Itemdict()
            self.items += copy_block.items
            # self.tokens = copy_block.tokens
            # self.free_spaces
            # self.aabbs

        elif boxtype is not None:
          if rot[0]=='w': self.l = boxtype.w
          elif rot[1]=='w': self.w = boxtype.w
          elif rot[2]=='w': self.h = boxtype.w

          if rot[0]=='l': self.l = boxtype.l
          elif rot[1]=='l': self.w = boxtype.l
          elif rot[2]=='l': self.h = boxtype.l        

          if rot[0]=='h': self.l = boxtype.h
          elif rot[1]=='h': self.w = boxtype.h
          elif rot[2]=='h': self.h = boxtype.h
          
          self.weight = boxtype.weight
          self.stacking_weight_resistance = boxtype.stacking_weight_resistance
          self.occupied_volume = boxtype.volume
          self.volume = boxtype.volume
          self.items = Itemdict()
          self.items[boxtype] = 1
          self.free_spaces = FreeSpace() # empty list of free spaces
          self.tokens = []

        else: 
          self.l = l; self.w = w; self.h = h
          self.occupied_volume = 0
          self.weight = weight
          self.stacking_weight_resistance = stacking_weight_resistance
          self.items = Itemdict()
          self.volume = l*w*h
          self.aabbs = []
          self.free_space = FreeSpace(Space(0, self.l, 0, self.w, 0, self.h, self)) # all is free space

    def add_block(self, block, space):
        x,y,z = space.corner_point
        if x == space.xmax: x -= block.l
        if y == space.ymax: y -= block.w
        if z == space.zmax: z -= block.h

        added_block = Aabb(x,x+block.l,y,y+block.w,z,z+block.h)
        added_block.weight = block.weight
        added_block.stacking_weight_resistance = block.stacking_weight_resistance

        # added_block.weight = block.stacking_weight_resistance

        self.aabbs.append(added_block)
        self.occupied_volume += block.occupied_volume
        self.weight += block.weight
        self.items += block.items
        self.free_space.crop(Aabb(x, x+block.l, y, y+block.w, z, z+block.h), self) # remove the space occupied by the block


    #x:w, y:l, z:h
    def join(self, block, dim, min_fr=0.98):
        if dim=='x':
            l = self.l + block.l; w = max(self.w, block.w); h = max(self.h, block.h); volume = w*l*h
            if (self.occupied_volume + block.occupied_volume)/volume < min_fr: return False
            self.weight += block.weight
            self.occupied_volume += block.occupied_volume
            self.volume = volume
            self.stacking_weight_resistance = min([self.stacking_weight_resistance,block.stacking_weight_resistance])
            self.items += block.items
        elif dim=='y':
            l = max(self.l, block.l); w = self.w + block.w; h = max(self.h, block.h); volume = w*l*h
            if (self.occupied_volume + block.occupied_volume)/volume < min_fr: return False
            self.weight += block.weight
            self.occupied_volume += block.occupied_volume
            self.stacking_weight_resistance = min([self.stacking_weight_resistance,block.stacking_weight_resistance])
            self.volume = volume
            self.items += block.items
        elif dim=='z':
            l = max(self.l, block.l); w = max(self.w, block.w); h = self.h + block.h; volume = w*l*h
            if (self.occupied_volume + block.occupied_volume)/volume < min_fr: return False
            self.weight += block.weight
            self.occupied_volume += block.occupied_volume
            self.stacking_weight_resistance = min([self.stacking_weight_resistance,block.stacking_weight_resistance])
            self.volume = volume
            self.items += block.items
        self.l = l; self.w = w; self.h = h

        return True

    def is_constructible(self, items):
        for item in self.items:
            if items[item] < self.items[item]:
                return False
        return True
    
    @staticmethod
    def generate_blocks(b1, b2, min_fr=0.98):
        a = copy(b1)
        if a.join(b2, 'x', min_fr): 
            yield a; a =  copy(b1)

        if a.join(b2, 'y', min_fr): 
            yield a; a =  copy(b1)

        if a.join(b2, 'z', min_fr): 
            yield a; 

    def occupied_volume_ratio(self):
        return self.occupied_volume/self.volume
    
    def __le__(self, other):
        return self.l <= other.l and self.w <= other.w and self.h <= other.h
      
    def __str__(self):
        return "Block: l: " + str(self.l) + " w: " + str(self.w) + " h: " + str(self.h) + " weight: " + str(self.weight)  + " stacking_weight_resistance: " + str(self.stacking_weight_resistance) + " volume: " + str(self.volume) + " occupied_volume: " + str(self.occupied_volume) + " items: " + str(self.items) + " ratio:" + str(self.occupied_volume_ratio())

class BlockList(list):
    def __init__(self, items, type, cont=None, min_fr=0.98, max_bl=10000, *args):
        super().__init__(*args)
        if(type=="simple_blocks"):
            self.generate_simple_blocks(items)

        if(type=="general_blocks"):
            self.generate_general_blocks(items,cont, min_fr, max_bl)

    def generate_general_blocks(self,items,cont,min_fr=0.98, max_bl=10000):
        self.generate_simple_blocks(items)
                                   
        B = self
        P = B.copy()

        while len(B) < max_bl:
            N = list()
            for b1 in P:
                for b2 in B:
                    for new_block in Block.generate_blocks(b1, b2, min_fr):
                        if new_block.is_constructible(items) and new_block <= cont:
                            N.append(new_block)
                            if len(B) + len(N) >= max_bl: break

                    if len(B) + len(N) >= max_bl: break

                if len(B) + len(N) >= max_bl: break

            if len(N) == 0: break
            B.extend(N)
            P = N


    def generate_simple_blocks(self,items):
        #items is a dictionary of boxtype->number
        for item in items:
            self.append(Block(item,"lwh"))
            if item.rot_l ==True:
                self.append(Block(item,"whl"))
                self.append(Block(item,"hwl"))
            
            if item.rot_w ==True:
                self.append(Block(item,"lhw"))
                self.append(Block(item,"hlw"))

            if item.rot_h ==True: # trivial
                #self.append(Block(item,"lwh"))
                self.append(Block(item,"wlh"))

    @staticmethod
    #porcentaje de area XY de block dentro de last, si block esta totalmente contenido dentro de last se retorna 1 (100%)
    def surface_percent(block, last):
        # Encuentra las coordenadas de intersección
        x_diff_max = max(block.xmin, last.xmin)
        x_diff_min = min(block.xmax, last.xmax)
        y_diff_max = max(block.ymin, last.ymin)
        y_diff_min = min(block.ymax, last.ymax)

        # Calcula el área de la intersección
        intersection_area = max(0, x_diff_min - x_diff_max) * max(0, y_diff_min - y_diff_max)

        # Calcula el área total de block
        block_area = (block.xmax - block.xmin) * (block.ymax - block.ymin)

        # Evita divisiones por cero y calcula el porcentaje
        if block_area == 0:
            return 0
        else:
            return intersection_area / block_area
    
    @staticmethod
    def blocks_on_top_list(aabb, aabbs):
        blocks_on_top = []
        for block in aabbs:
            # Verificar si 'block' está encima de 'aabb' en el eje Z
            if block.zmin >= aabb.zmax:
                # Verificar la intersección en el plano XY
                if (
                    (block.xmin <= aabb.xmin <= block.xmax or block.xmin <= aabb.xmax <= block.xmax) and
                    (block.ymin <= aabb.ymin <= block.ymax or block.ymin <= aabb.ymax <= block.ymax)
                ):
                    blocks_on_top.append(block)
        return blocks_on_top


    @staticmethod
    def blocks_weight_supported(p_block ,aabbs, space):
        #en este punto pblock contiene el peso del bloque y la resistencia del peso
        p_blocks_supported = []
        # Cada bloque posible
        for block in p_block:
            x,y,z = space.corner_point
            if x == space.xmax: x -= block.l
            if y == space.ymax: y -= block.w
            if z == space.zmax: z -= block.h
            
            block_test = Aabb(x,x+block.l,y,y+block.w,z,z+block.h)
            block_test.weight = block.weight
            block_test.stacking_weight_resistance = block.stacking_weight_resistance
            block_is_valid = True

            # Almacenar aabbs que estan debajo del bloque posible
            under_blocks = []
            for aabb in aabbs:
                # print(aabb)
                # Si el aabb esta debajo del block_test en Z e intersectando en la dimension XY, se agrega
                if (aabb.zmax <= block_test.zmin):
                    if ((block_test.xmin <= aabb.xmin <= block_test.xmax or block_test.xmin <= aabb.xmax <= block_test.xmax) and
                        (block_test.ymin <= aabb.ymin <= block_test.ymax or block_test.ymin <= aabb.ymax <= block_test.ymax)): 
                        under_blocks.append(aabb)
                        # if( block_test.zmin != 0):
                        #     print(aabb)

            #en este punto tengo que bloque que quiero probar y una lista de los aabbs que estan debajo de el
            #evaluar como afecta poner el block_test para todo los bloques que esten debajo de el
            #aabbs contiene los bloques del container

            stacking_weight_is_valid = True

            for underblock in under_blocks:
                #buscar si bloques que esten arriba * superficie + block_test * superficie no supera la capacidad del underblock
                blocks_higher = BlockList.blocks_on_top_list(underblock, aabbs)

                total_weight_overlapping = 0
                #validar bloques que estan en la base y en la cima de otros
                for block_h in blocks_higher:
                    surface_percent_block = BlockList.surface_percent(block_h, underblock)
                    total_weight_overlapping = total_weight_overlapping + block_h.weight * surface_percent_block

                #Si algun bloque de abajo no es capaz de soportar el nuevo bloque, entonces no cumple con restriccion
                if( total_weight_overlapping + block_test.weight * BlockList.surface_percent(block_test,underblock) >= underblock.stacking_weight_resistance):
                    block_is_valid = False
                    
            if ( block_is_valid ):
                p_blocks_supported.append(block)

        return p_blocks_supported

    def possible_blocks(blocks,maxL,maxW,maxH, container, space, weight_restriction=False): #, weight_restriction=True): 
        a = 0
        p_block = []
        # print("POSIBLE BLOCKS")
        for block in blocks:
            if block.w <= maxW and block.l <= maxL and block.h <= maxH:
                p_block.append(block)

        p_block_w_supported = blocks.blocks_weight_supported(p_block,container.aabbs, space)
        # print("posible blocks:", len(p_block))
        # print("posible blocks que cumplen restriccion de peso:", len(p_block_w_supported))
        # print("--------------")

        import csv
        csv_filename = "HISTORIAL_SALIDA_APILAMIENTO.csv"
        # print("creando archivo HISTORIAL_SALIDA_APILAMIENTO.csv")

        with open(csv_filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            # csv_writer.writerow(["bloques posibles", "cumplen"])
            # Escribir datos
            csv_writer.writerow([len(p_block), len(p_block_w_supported)])


        if weight_restriction: return p_block_w_supported #aqui se activa la restriccion de apilamiento

        # if weight_restriction:
        #     p_block_w_supported = blocks.blocks_weight_supported(p_block,container.aabbs, space)
        #     print("posible blocks:", len(p_block))
        #     print("posible blocks que cumplen restriccion de peso:", len(p_block_w_supported))
        #     print("--------------")
        #     return p_block_w_supported

        return p_block

    def largest(blocks, maxL, maxW, maxH):
        largest = None
        for block in blocks:
            if block.w <= maxW and block.l <= maxL and block.h <= maxH:
                if largest is None or block.volume > largest.volume:
                    largest = block
        return largest


    #remove blocks that cannot be constructed with the given items(boxtype->number)
    def remove_unconstructable(blocks, items):
        #cannot remove while iterating
        to_remove = list()
        for block in blocks:
            if not block.is_constructible(items):
                to_remove.append(block)
        for block in to_remove:
            blocks.remove(block)

    def __str__(self):
        _str = ""
        for block in self:
            _str+= str(block) + "\n"



