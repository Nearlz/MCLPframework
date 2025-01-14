from plotly.offline import init_notebook_mode, iplot
import plotly.graph_objs as go
from random import random
from typing import Tuple
import numpy as np


class Box:

    x: Tuple[float, float]
    y: Tuple[float, float]
    z: Tuple[float, float]

    i: float
    j: float
    k: float
        
  
    def __init__(self, dim):
        dim = np.array(dim)
        dim.shape = (2,3)
        self.init(dim)

    def init(self, dim):
        # Agregando puntos
        points = []
        points.append((dim[0][0], dim[0][1], dim[0][2]))
        points.append((dim[0][0], dim[1][1], dim[0][2]))
        points.append((dim[1][0], dim[1][1], dim[0][2]))
        points.append((dim[1][0], dim[0][1], dim[0][2]))
        points.append((dim[0][0], dim[0][1], dim[1][2]))
        points.append((dim[0][0], dim[1][1], dim[1][2]))
        points.append((dim[1][0], dim[1][1], dim[1][2]))
        points.append((dim[1][0], dim[0][1], dim[1][2]))

        self.x = []
        self.y = []
        self.z = []

        for p in points:
            self.x.append(p[0])
            self.y.append(p[1])
            self.z.append(p[2])

        self.i = [7, 0, 0, 0, 4, 4, 2, 6, 4, 0, 3, 7]
        self.j = [3, 4, 1, 2, 5, 6, 5, 5, 0, 1, 2, 2]
        self.k = [0, 7, 2, 3, 6, 7, 1, 2, 5, 5, 7, 6]

def plot_container(cont_dim, box_dims):
    boxes = []
    for box_dim in box_dims:
        boxes.append(Box(box_dim))

    data = []

    #para ir agregando colores en concreto, debe modificarse tracel y recibir un numero concreto de boxes
    colors=[132,182,244,
            225,192,182,
            17,153,136]
    
    # for box in boxes:
    for index, box in enumerate(boxes):
        opacity_value = 0 if index == len(boxes) - 1 else 1
        tracel = go.Mesh3d(
                  x=box.x, 
                  y=box.y, 
                  z=box.z, 
                  i=box.i,
                  j=box.j, 
                  k=box.k, 
                  opacity= opacity_value,
                  color='rgb({},{},{})'.format(
                    #   colors[0+3*index],colors[1+3*index],colors[2+3*index]
                      random()*255,
                      random()*255,
                      random()*255
                  )
              )
        data.append(tracel)

    # Configurando layout del grafico
    layout = go.Layout(
            autosize=False,
            margin=dict(
                l=0,
                r=0,
                b=0,
                t=0)
            )

    fig = go.Figure(data=data, layout=layout)

    # Configurando grafico
    fig.update_layout(
        paper_bgcolor='rgba(0,0,0,0)',
        scene=dict(
            xaxis=dict(nticks=10, range=[0, cont_dim[0]], ),
            yaxis=dict(nticks=5, range=[0, cont_dim[1]], ),
            zaxis=dict(nticks=5, tickfont=dict(size=15), range=[0, cont_dim[2]], ),
            xaxis_showspikes=False,
            yaxis_showspikes=False,
            camera=dict(up=dict(x=0, y=0, z=1),
                      center=dict(x=0, y=0, z=0),
                        eye=dict(x=-2, y=2, z=1.5))
        ),
    )

    # Renderizando grafico
    iplot(fig, filename="holo")
    #fig.show()