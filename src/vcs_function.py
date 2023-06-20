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
            
            if abs(block.xmax - last.xmin) <= (block.xmax - block.xmin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.ymin < last.ymax and block.ymax > last.ymin )):
                z_diff_max = max([block.zmin,last.zmin])
                z_diff_min = min([block.zmax,last.zmax])
                y_diff_max = max([block.ymin,last.ymin])
                y_diff_min = min([block.ymax,last.ymax])
                block_value[i]+= (z_diff_min-z_diff_max) * (y_diff_min-y_diff_max)

            elif abs(block.xmin - last.xmax) <= (block.xmax - block.xmin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.ymin < last.ymax and block.ymax > last.ymin )):
                z_diff_max = max([block.zmin,last.zmin])
                z_diff_min = min([block.zmax,last.zmax])
                y_diff_max = max([block.ymin,last.ymin])
                y_diff_min = min([block.ymax,last.ymax])
                block_value[i]+= (z_diff_min-z_diff_max) * (y_diff_min-y_diff_max)

            elif abs(block.ymax - last.ymin) <= (block.ymax - block.ymin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                z_diff_max = max([block.zmin,last.zmin])
                z_diff_min = min([block.zmax,last.zmax])
                x_diff_max = max([block.xmin,last.xmin])
                x_diff_min = min([block.xmax,last.xmax])
                block_value[i]+= (z_diff_min-z_diff_max) * (x_diff_min-x_diff_max)

            elif abs(block.ymin - last.ymax) <= (block.ymax - block.ymin) * p and ((block.zmin < last.zmax and block.zmax > last.zmin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                z_diff_max = max([block.zmin,last.zmin])
                z_diff_min = min([block.zmax,last.zmax])
                x_diff_max = max([block.xmin,last.xmin])
                x_diff_min = min([block.xmax,last.xmax])
                block_value[i]+= (z_diff_min-z_diff_max) * (x_diff_min-x_diff_max)

            elif abs(block.zmax - last.zmin) <= (block.zmax - block.zmin) * p and ((block.ymin < last.ymax and block.ymax > last.ymin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                y_diff_max = max([block.ymin,last.ymin])
                y_diff_min = min([block.ymax,last.ymax])
                x_diff_max = max([block.xmin,last.xmin])
                x_diff_min = min([block.xmax,last.xmax])
                block_value[i]+= (y_diff_min-y_diff_max) * (x_diff_min-x_diff_max)

            elif abs(block.zmin - last.zmax) <= (block.zmax - block.zmin) * p and ((block.ymin < last.ymax and block.ymax > last.ymin ) and (block.xmin < last.xmax and block.xmax > last.xmin )):
                y_diff_max = max([block.ymin,last.ymin])
                y_diff_min = min([block.ymax,last.ymax])
                x_diff_max = max([block.xmin,last.xmin])
                x_diff_min = min([block.xmax,last.xmax])
                block_value[i]+= (y_diff_min-y_diff_max) * (x_diff_min-x_diff_max)

            elif block.xmin <= (block.xmax - block.xmin) * p or abs(block.xmax - space.l) <= (block.xmax - block.xmin) * p:
                block_value[i]+= (block.ymax-block.ymin) * (block.zmax-block.zmin)
            elif block.ymin <= (block.ymax - block.ymin) * p or abs(block.ymax - space.w) <= (block.ymax - block.ymin) * p:
                block_value[i]+= (block.xmax-block.xmin) * (block.zmax-block.zmin)
            elif block.zmin <= (block.zmax - block.zmin) * p or abs(block.zmax - space.h) <= (block.zmax - block.zmin) * p:
                block_value[i]+= (block.ymax-block.ymin) * (block.xmax-block.xmin)

    max_index = block_value.index(max(block_value))
    return blocks[max_index]

def eval_function(blocks,space,stored_blocks, p:float, test:bool = False) :
    if test == True:
        CS_function(blocks,space,p,stored_blocks)
    else:
        p_blocks = blocks.possible_blocks(space.l, space.w, space.h)
        return CS_function(p_blocks,space,p,stored_blocks)
        # return blocks.largest(space.l, space.w, space.h)
