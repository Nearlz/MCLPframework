from .base import Block,Aabb
import math

def center_of_gravity(container: Block,p_blocks: list = None,space: Aabb = None):

    if p_blocks is not None and space is not None:
        distance = []

        for possible_block in p_blocks:

            x,y,z = space.corner_point
            if x == space.xmax: x -= possible_block.l
            if y == space.ymax: y -= possible_block.w
            if z == space.zmax: z -= possible_block.h

            block = Aabb(x,x+possible_block.l,y,y+possible_block.w,z,z+possible_block.h)
            block.weight = possible_block.weight

            total_mass = 0
            center_x = 0
            center_y = 0

            for aabb in container.aabbs:
                total_mass += aabb.weight

                # Calculate the center of the Aabb in each axis
                center_x += (aabb.xmin + aabb.xmax) / 2 * aabb.weight
                center_y += (aabb.ymin + aabb.ymax) / 2 * aabb.weight

            #Calculate the center for the possible block
            total_mass += block.weight

            center_x += (block.xmin + block.xmax) / 2 * block.weight
            center_y += (block.ymin + block.ymax) / 2 * block.weight

            # Calculate the center of gravity by dividing by the total mass
            center_x /= total_mass
            center_y /= total_mass

            # Calculate the distance difference
            container_center_x = container.l / 2
            container_center_y = container.w / 2

            distance.append(math.sqrt((center_x - container_center_x)**2 + (center_y - container_center_y)**2))
            
    else:

        distance = 0

        total_mass = 0
        center_x = 0
        center_y = 0

        for aabb in container.aabbs:
            total_mass+= aabb.weight

            # Calculate the center of the Aabb in each axis
            center_x += (aabb.xmin + aabb.xmax) / 2 * aabb.weight
            center_y += (aabb.ymin + aabb.ymax) / 2 * aabb.weight

        # Calculate the center of gravity by dividing by the total mass
        center_x /= total_mass
        center_y /= total_mass

        # Calculate the distance difference
        container_center_x = container.l / 2
        container_center_y = container.w / 2

        distance = math.sqrt((center_x - container_center_x)**2 + (center_y - container_center_y)**2)
    return distance

def distance_cg_is_valid(distance, slack):
    if distance <= slack:
        return True
    return False


def max_weight_supported(containers: list[Block], max_supported_by_container: int) -> bool:
    total_weight = 0
    for container in containers:
        total_weight += container.weight
    
    if total_weight > max_supported_by_container:
        return False
    
    return True

def stacked_weght(block: Block, container: Block):
    pass