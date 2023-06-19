def CS_function(blocks,space):
    pass

def eval_function(blocks,space,test = False):
    if test == True:
        CS_function(blocks,space)
    else:
        CS_function(blocks,space)
        return blocks.largest(space.l, space.w, space.h)
