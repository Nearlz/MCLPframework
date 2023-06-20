from .base import Block
import math


def center_of_gravity(container: Block):
    total_mass = 0
    center_x = 0
    center_y = 0
    center_z = 0

    for aabb in container.aabbs:
        aabb_mass = aabb.volume
        total_mass += aabb_mass

        # Calculate the center of the Aabb in each axis
        center_x += (aabb.xmin + aabb.xmax) / 2 * aabb_mass
        center_y += (aabb.ymin + aabb.ymax) / 2 * aabb_mass
        center_z += (aabb.zmin + aabb.zmax) / 2 * aabb_mass

    # Calculate the center of gravity by dividing by the total mass
    center_x /= total_mass
    center_y /= total_mass
    center_z /= total_mass

    # Calculate the distance difference
    container_center_x = container.l / 2
    container_center_y = container.w / 2
    container_center_z = container.h / 2

    distance = math.sqrt((center_x - container_center_x)**2 + (center_y - container_center_y)**2 + (center_z - container_center_z)**2)

    return distance

def max_weight_suppoted(container: Block):
    pass

def stacked_weght(block: Block, container: Block):
    pass