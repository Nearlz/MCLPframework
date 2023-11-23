from .base import Aabb
from .restrictions import center_of_gravity
import numpy as np
import pandas as pd
import copy
from src.base import Block

def CS_function(blocks,space,p,container):
    stored_blocks = container.aabbs
    block_value = [0] * len(blocks)
    for i,possible_block in enumerate(blocks,0):
        x,y,z = space.corner_point
        if x == space.xmax: x -= possible_block.l
        if y == space.ymax: y -= possible_block.w
        if z == space.zmax: z -= possible_block.h
        block = Aabb(x,x+possible_block.l,y,y+possible_block.w,z,z+possible_block.h)

        for last in stored_blocks:
            x_diff_max = max([block.xmin,last.xmin])
            x_diff_min = min([block.xmax,last.xmax])
            y_diff_max = max([block.ymin,last.ymin])
            y_diff_min = min([block.ymax,last.ymax])
            z_diff_max = max([block.zmin,last.zmin])
            z_diff_min = min([block.zmax,last.zmax])
            
            if abs(block.xmax - last.xmin) <= (block.xmax - block.xmin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.ymin < last.ymax and block.ymax > last.ymin )):
                block_value[i]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

            elif abs(block.xmin - last.xmax) <= (block.xmax - block.xmin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.ymin < last.ymax and block.ymax > last.ymin )):
                block_value[i]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

            elif abs(block.ymax - last.ymin) <= (block.ymax - block.ymin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                block_value[i]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

            elif abs(block.ymin - last.ymax) <= (block.ymax - block.ymin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                block_value[i]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

            elif abs(block.zmax - last.zmin) <= (block.zmax - block.zmin) * p and ((block.ymin < last.ymax and block.ymax > last.ymin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                block_value[i]+= ((y_diff_min-y_diff_max) * (x_diff_min-x_diff_max))

            elif abs(block.zmin - last.zmax) <= (block.zmax - block.zmin) * p and ((block.ymin < last.ymax and block.ymax > last.ymin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                block_value[i]+= ((y_diff_min-y_diff_max) * (x_diff_min-x_diff_max))

        if block.xmin <= (block.xmax - block.xmin) * p or abs(block.xmax - container.l) <= (block.xmax - block.xmin) * p:
            block_value[i]+= ((block.ymax-block.ymin) * (block.zmax-block.zmin))
        if block.ymin <= (block.ymax - block.ymin) * p or abs(block.ymax - container.w) <= (block.ymax - block.ymin) * p:
            block_value[i]+= ((block.xmax-block.xmin) * (block.zmax-block.zmin))
        if block.zmin <= (block.zmax - block.zmin) * p or abs(block.zmax - container.h) <= (block.zmax - block.zmin) * p:
            block_value[i]+= ((block.ymax-block.ymin) * (block.xmax-block.xmin))

        block_value[i]/=(possible_block.l * possible_block.w) * 2 + (possible_block.l * possible_block.h) * 2 + (possible_block.w * possible_block.h) * 2
    
    return block_value


def dynamic_stability(blocks,space,gap,container,final=False):
    tolerance = 0.5
    stored_blocks = container.aabbs
    if final:
        block_value = 0
        stored_blocks_surfaces = np.zeros((len(stored_blocks),4))
        stored_blocks_values = np.zeros(len(stored_blocks))
        for j,first_block in enumerate(stored_blocks):
            for k in range(j+1,len(stored_blocks)):
                second_block = stored_blocks[k]
                x_diff_max = max([second_block.xmin,first_block.xmin])
                x_diff_min = min([second_block.xmax,first_block.xmax])
                y_diff_max = max([second_block.ymin,first_block.ymin])
                y_diff_min = min([second_block.ymax,first_block.ymax])
                z_diff_max = max([second_block.zmin,first_block.zmin])
                z_diff_min = min([second_block.zmax,first_block.zmax])
                
                if (abs(second_block.xmax - first_block.xmin) <= (second_block.xmax - first_block.xmin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.ymin < first_block.ymax and second_block.ymax > first_block.ymin )):
                    stored_blocks_surfaces[k,0]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))
                    stored_blocks_surfaces[j,1]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

                elif (abs(second_block.xmin - first_block.xmax) <= (second_block.xmax - first_block.xmin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.ymin < first_block.ymax and second_block.ymax > first_block.ymin )):
                    stored_blocks_surfaces[k,1]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))
                    stored_blocks_surfaces[j,0]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

                elif (abs(second_block.ymax - first_block.ymin) <= (second_block.ymax - first_block.ymin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.xmin < first_block.xmax and second_block.xmax > first_block.xmin )):
                    stored_blocks_surfaces[k,2]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))
                    stored_blocks_surfaces[j,3]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

                elif (abs(second_block.ymin - first_block.ymax) <= (second_block.ymax - first_block.ymin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.xmin < first_block.xmax and second_block.xmax > first_block.xmin )):
                    stored_blocks_surfaces[k,3]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))
                    stored_blocks_surfaces[j,2]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

            if first_block.xmin <= (first_block.xmax - first_block.xmin) * gap:
                stored_blocks_surfaces[j,0]+= ((first_block.ymax-first_block.ymin) * (first_block.zmax-first_block.zmin))
            if abs(first_block.xmax - container.l) <= (first_block.xmax - first_block.xmin) * gap:
                stored_blocks_surfaces[j,1]+= ((first_block.ymax-first_block.ymin) * (first_block.zmax-first_block.zmin))
            if first_block.ymin <= (first_block.ymax - first_block.ymin) * gap:
                stored_blocks_surfaces[j,2]+= ((first_block.xmax-first_block.xmin) * (first_block.zmax-first_block.zmin))
            if abs(first_block.ymax - container.w) <= (first_block.ymax - first_block.ymin) * gap:
                stored_blocks_surfaces[j,3]+= ((first_block.xmax-first_block.xmin) * (first_block.zmax-first_block.zmin))


        for i in range(len(stored_blocks_surfaces)):
            if (stored_blocks_surfaces[i,0] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].ymax - stored_blocks[i].ymin))) > tolerance:
                stored_blocks_values[i]+=1
            if (stored_blocks_surfaces[i,1] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].ymax - stored_blocks[i].ymin))) > tolerance:
                stored_blocks_values[i]+=1
            if (stored_blocks_surfaces[i,2] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].xmax - stored_blocks[i].xmin))) > tolerance:
                stored_blocks_values[i]+=1
            if (stored_blocks_surfaces[i,3] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].xmax - stored_blocks[i].xmin))) > tolerance:
                stored_blocks_values[i]+=1
            if stored_blocks_values[i] >= 3:
                block_value += (stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].ymax - stored_blocks[i].ymin) * (stored_blocks[i].xmax - stored_blocks[i].xmin)


    else:
        block_value = np.zeros(len(blocks))
        stored_blocks_surfaces = np.zeros((len(stored_blocks)+1,4))
        stored_blocks_values = np.zeros(len(stored_blocks)+1)
        #Este FOR evalua todos con todos dentro del contenedor, antes de considerar los bloques posibles
        # for j in range(len(stored_blocks)):
        # first_block = stored_blocks[j].copy()
        # for k in range(j+1,len(stored_blocks)):
        for j,first_block in enumerate(stored_blocks):
            for k in range(j+1,len(stored_blocks)):
                second_block = stored_blocks[k]
                x_diff_max = max([second_block.xmin,first_block.xmin])
                x_diff_min = min([second_block.xmax,first_block.xmax])
                y_diff_max = max([second_block.ymin,first_block.ymin])
                y_diff_min = min([second_block.ymax,first_block.ymax])
                z_diff_max = max([second_block.zmin,first_block.zmin])
                z_diff_min = min([second_block.zmax,first_block.zmax])
                
                if (abs(second_block.xmax - first_block.xmin) <= (second_block.xmax - first_block.xmin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.ymin < first_block.ymax and second_block.ymax > first_block.ymin )):
                    stored_blocks_surfaces[k,0]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))
                    stored_blocks_surfaces[j,1]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

                elif (abs(second_block.xmin - first_block.xmax) <= (second_block.xmax - first_block.xmin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.ymin < first_block.ymax and second_block.ymax > first_block.ymin )):
                    stored_blocks_surfaces[k,1]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))
                    stored_blocks_surfaces[j,0]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

                elif (abs(second_block.ymax - first_block.ymin) <= (second_block.ymax - first_block.ymin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.xmin < first_block.xmax and second_block.xmax > first_block.xmin )):
                    stored_blocks_surfaces[k,2]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))
                    stored_blocks_surfaces[j,3]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

                elif (abs(second_block.ymin - first_block.ymax) <= (second_block.ymax - first_block.ymin) * gap) and ((second_block.zmin < first_block.zmax and second_block.zmax > first_block.zmin ) and (second_block.xmin < first_block.xmax and second_block.xmax > first_block.xmin )):
                    stored_blocks_surfaces[k,3]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))
                    stored_blocks_surfaces[j,2]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

            if first_block.xmin <= (first_block.xmax - first_block.xmin) * gap:
                stored_blocks_surfaces[j,0]+= ((first_block.ymax-first_block.ymin) * (first_block.zmax-first_block.zmin))
            if abs(first_block.xmax - container.l) <= (first_block.xmax - first_block.xmin) * gap:
                stored_blocks_surfaces[j,1]+= ((first_block.ymax-first_block.ymin) * (first_block.zmax-first_block.zmin))
            if first_block.ymin <= (first_block.ymax - first_block.ymin) * gap:
                stored_blocks_surfaces[j,2]+= ((first_block.xmax-first_block.xmin) * (first_block.zmax-first_block.zmin))
            if abs(first_block.ymax - container.w) <= (first_block.ymax - first_block.ymin) * gap:
                stored_blocks_surfaces[j,3]+= ((first_block.xmax-first_block.xmin) * (first_block.zmax-first_block.zmin))

        for j,possible_block in enumerate(blocks):
            block_surfaces = stored_blocks_surfaces.copy()
            stored_blocks_values = np.zeros(len(stored_blocks)+1)
            x,y,z = space.corner_point
            if x == space.xmax: x -= possible_block.l
            if y == space.ymax: y -= possible_block.w
            if z == space.zmax: z -= possible_block.h
            block = Aabb(x,x+possible_block.l,y,y+possible_block.w,z,z+possible_block.h)
            for i,stored_block in enumerate(stored_blocks):
                x_diff_max = max([stored_block.xmin,block.xmin])
                x_diff_min = min([stored_block.xmax,block.xmax])
                y_diff_max = max([stored_block.ymin,block.ymin])
                y_diff_min = min([stored_block.ymax,block.ymax])
                z_diff_max = max([stored_block.zmin,block.zmin])
                z_diff_min = min([stored_block.zmax,block.zmax])
                if abs(stored_block.xmax - block.xmin) <= abs(stored_block.xmax - block.xmin) * gap and ((stored_block.zmin < block.zmax and stored_block.zmax > block.zmin ) and (stored_block.ymin < block.ymax and stored_block.ymax > block.ymin )):
                    block_surfaces[i,0]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))
                    block_surfaces[-1,1]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

                elif abs(stored_block.xmin - block.xmax) <= abs(stored_block.xmax - block.xmin) * gap and ((stored_block.zmin < block.zmax and stored_block.zmax > block.zmin ) and (stored_block.ymin < block.ymax and stored_block.ymax > block.ymin )):
                    block_surfaces[i,1]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))
                    block_surfaces[-1,0]+= ((z_diff_min-z_diff_max) * (y_diff_min-y_diff_max))

                elif abs(stored_block.ymax - block.ymin) <= abs(stored_block.ymax - block.ymin) * gap and ((stored_block.zmin < block.zmax and stored_block.zmax > block.zmin ) and (stored_block.xmin < block.xmax and stored_block.xmax > block.xmin )):
                    block_surfaces[i,2]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))
                    block_surfaces[-1,3]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

                elif abs(stored_block.ymin - block.ymax) <= abs(stored_block.ymax - block.ymin) * gap and ((stored_block.zmin < block.zmax and stored_block.zmax > block.zmin ) and (stored_block.xmin < block.xmax and stored_block.xmax > block.xmin )):
                    block_surfaces[i,3]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))
                    block_surfaces[-1,2]+= ((z_diff_min-z_diff_max) * (x_diff_min-x_diff_max))

            if block.xmin <= (block.xmax - block.xmin) * gap:
                block_surfaces[-1,0]+= ((block.ymax-block.ymin) * (block.zmax-block.zmin))
            if abs(block.xmax - container.l) <= (block.xmax - block.xmin) * gap:
                block_surfaces[-1,1]+= ((block.ymax-block.ymin) * (block.zmax-block.zmin))
            if block.ymin <= (block.ymax - block.ymin) * gap:
                block_surfaces[-1,2]+= ((block.xmax-block.xmin) * (block.zmax-block.zmin))
            if abs(block.ymax - container.w) <= (block.ymax - block.ymin) * gap:
                block_surfaces[-1,3]+= ((block.xmax-block.xmin) * (block.zmax-block.zmin))

            for i in range(len(block_surfaces)-1):
                if (block_surfaces[i,0] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].ymax - stored_blocks[i].ymin))) > tolerance:
                    stored_blocks_values[i]+=1
                if (block_surfaces[i,1] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].ymax - stored_blocks[i].ymin))) > tolerance:
                    stored_blocks_values[i]+=1
                if (block_surfaces[i,2] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].xmax - stored_blocks[i].xmin))) > tolerance:
                    stored_blocks_values[i]+=1
                if (block_surfaces[i,3] / ((stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].xmax - stored_blocks[i].xmin))) > tolerance:
                    stored_blocks_values[i]+=1
                if stored_blocks_values[i] >= 3:
                    block_value[j] += (stored_blocks[i].zmax - stored_blocks[i].zmin) * (stored_blocks[i].ymax - stored_blocks[i].ymin) * (stored_blocks[i].xmax - stored_blocks[i].xmin)

            if (block_surfaces[-1,0] / ((block.zmax - block.zmin) * (block.ymax - block.ymin))) > tolerance:
                stored_blocks_values[-1]+=1
            if (block_surfaces[-1,1] / ((block.zmax - block.zmin) * (block.ymax - block.ymin))) > tolerance:
                stored_blocks_values[-1]+=1
            if (block_surfaces[-1,2] / ((block.zmax - block.zmin) * (block.xmax - block.xmin))) > tolerance:
                stored_blocks_values[-1]+=1
            if (block_surfaces[-1,3] / ((block.zmax - block.zmin) * (block.xmax - block.xmin))) > tolerance:
                stored_blocks_values[-1]+=1
            if stored_blocks_values[-1] >= 3:
                block_value[j] += (block.zmax - block.zmin) * (block.ymax - block.ymin) * (block.xmax - block.xmin)
            block_value[j] /= container.l * container.h * container.w

    return block_value
        


def maximize_axis(limit,items):
    items.sort(reverse=True)

    positions = []
    # Iterate over each item and find a suitable position in the container
    for item in items:
        # Check if the item can fit in any existing bin
        for i, position in enumerate(positions):
            if position + item <= limit:
                # Pack the item into the current bin
                positions[i] += item
                break
            elif item <= limit:
            # If no suitable bin is found, create a new bin and pack the item
                positions.append(item)

    if positions:
        return max(positions)
    else:
        return 0

def loss_function(p_blocks,space,items):

    V_loss = []

    for possible_block in p_blocks:
        x,y,z = space.corner_point
        if x == space.xmax: x -= possible_block.l
        if y == space.ymax: y -= possible_block.w
        if z == space.zmax: z -= possible_block.h

        block = Aabb(x,x+possible_block.l,y,y+possible_block.w,z,z+possible_block.h)

        difference = space.l - (block.xmax - block.xmin)
        aux_items = [x.l for x in items]
        l_max = maximize_axis(difference,aux_items)

        difference = space.w - (block.ymax - block.ymin)
        aux_items = [x.w for x in items]
        w_max = maximize_axis(difference,aux_items)

        difference = space.w - (block.zmax - block.zmin)
        aux_items = [x.h for x in items]
        h_max = maximize_axis(difference,aux_items)

        aux = (((block.xmax - block.xmin) + l_max ) * ((block.ymax - block.ymin) + w_max) * ((block.zmax - block.zmin) + h_max))

        V_i = space.volume - aux

        V_i = V_i / space.volume

        V_loss.append(V_i)
    return V_loss

def n(p_blocks):
    n_boxes = []
    for block in p_blocks:
        n_box = sum(block.items.values())
        n_boxes.append(float(n_box))
    return n_boxes

def eval_function(blocks,space,container, params,items, weight_restriction=False, max_distance_cg = -1) :
    p_blocks = blocks.possible_blocks(space.l, space.w, space.h, container, space, weight_restriction)

    if len(p_blocks) == 0: return None #si no existen bloques posibles devuelve None para seguir con otro espacio

    alpha = params[0]; beta = params[1]; gamma = params[2]; delta = params[3]; p = params[4]

    V = [(x.l * x.w * x.l) for x in p_blocks]

    resistance = [x.stacking_weight_resistance for x in p_blocks]

    K = 1 - ((container.volume - container.occupied_volume) / container.volume)
    # print("constante k: ", K )

    V_loss = loss_function(p_blocks,space,items)

    CS = CS_function(p_blocks,space,p,container=container)

    gap = 0.05

    DS = dynamic_stability(p_blocks,space,gap=gap,container=container)
    # DS_df = pd.DataFrame(DS)
    # DS_df.to_csv(f'DS_data_{len(DS)}.csv')
    # print(len(DS))

    N_b = n(p_blocks)

    CG = center_of_gravity(container,p_blocks,space)
    
    pha = 0.6

    # print("centro de gravedad:", CG)

    total_old = [w * x**alpha * y**beta * z**-gamma * (1-1/(1+a**delta)) for w, x, y, z, a in zip(V,CS,V_loss,N_b, CG)]

    total = [w * x**alpha * (1-y)**beta * z**-gamma *  (b**delta) * (a ** K) * (c**pha) for w, x, y, z, a, b, c in zip(V,CS,V_loss,N_b,DS, CG,resistance)]

    # print("total old: ", total_old)
    # print("total    : ", total)

    index = total.index(max(total))
    # aux_container = container
    aux_container = Block(l=container.l,w=container.w,h=container.h,weight=container.w,stacking_weight_resistance=container.stacking_weight_resistance)
    aux_container.aabbs = container.aabbs.copy()
    aux_container.add_block(p_blocks[index], space)
    items -= p_blocks[index].items
    blocks.remove_unconstructable(items)
    aux_container.free_space.filter(items)
    aux_space = aux_container.free_space.closest_space()

    # print("aux space: ", aux_space)
    
    if aux_space is None:
        # print("AUX SPACE IS NONEEEEEEEEEEEEEEEE")
        import csv
        csv_filename = "HISTORIAL_SALIDA_APILAMIENTO.csv"
        # Abrir el archivo CSV y agregar "Aux space NONE" en una nueva fila
        with open(csv_filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["Aux space NONE"])
        
    #encontrar el bloque posible con mayor total que no incumpla restriccion de distancia maxima al CG
    while( max_distance_cg < CG[index] and max_distance_cg >= 0 ):

        # print("centro de gravedad del best:",center_of_gravity(container,p_blocks,space))
        # print(p_blocks[index])
        # print("len pblocks: ",len(p_blocks))
        # print("len cg     : ",len(CG))
        # print("len total  : ",len(total))

        CG.pop(index)
        p_blocks.pop(index)
        total.pop(index)

        if( len(p_blocks) > 0 ):
            index = total.index(max(total))
        else:
            return None #No existe solucion que cumpla con el centro de gravedad
        

    return p_blocks[index]

    # return blocks.largest(space.l, space.w, space.h)
