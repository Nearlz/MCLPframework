from .base import Aabb

def CS_function(blocks,space,p,stored_blocks):
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

            elif block.xmin <= (block.xmax - block.xmin) * p or abs(block.xmax - space.l) <= (block.xmax - block.xmin) * p:
                block_value[i]+= ((block.ymax-block.ymin) * (block.zmax-block.zmin))
            elif block.ymin <= (block.ymax - block.ymin) * p or abs(block.ymax - space.w) <= (block.ymax - block.ymin) * p:
                block_value[i]+= ((block.xmax-block.xmin) * (block.zmax-block.zmin))
            elif block.zmin <= (block.zmax - block.zmin) * p or abs(block.zmax - space.h) <= (block.zmax - block.zmin) * p:
                block_value[i]+= ((block.ymax-block.ymin) * (block.xmax-block.xmin))
    
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

def loss_function(p_blocks,space,stored_blocks,items):

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


def eval_function(blocks,space,stored_blocks, p:float,items) :
    p_blocks = blocks.possible_blocks(space.l, space.w, space.h)

    # Nb = [x.items for x in p_blocks if len(x.items) != 1]
    # print(Nb)

    V = [(x.l * x.w * x.l) for x in p_blocks]

    V_loss = loss_function(p_blocks,space,stored_blocks,items)

    CS = CS_function(p_blocks,space,p,stored_blocks)

    print(f'{len(V)}  {len(V_loss)}  {len(CS)}')

    total = [x * y * z for x, y, z in zip(V,V_loss,CS)]

    index = total.index(max(total))

    return p_blocks[index]

    # return blocks.largest(space.l, space.w, space.h)
