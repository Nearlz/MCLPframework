from .base import Block

def center_of_gravity(container: Block):
    total_mass = 0
    center_x = 0
    center_y = 0
    center_z = 0

    for aabb in container.aabbs:
        aabb_mass = aabb.volume
        total_mass += aabb_mass

        # Calcula el centro del Aabb en cada eje
        center_x += (aabb.xmin + aabb.xmax) / 2 * aabb_mass
        center_y += (aabb.ymin + aabb.ymax) / 2 * aabb_mass
        center_z += (aabb.zmin + aabb.zmax) / 2 * aabb_mass

    # Calcula el centro de gravedad dividiendo por la masa total
    center_x /= total_mass
    center_y /= total_mass
    center_z /= total_mass

    return (center_x, center_y, center_z)

def max_weight_suppoted(container: Block):
    pass

def stacked_weght(block: Block, container: Block):
    pass