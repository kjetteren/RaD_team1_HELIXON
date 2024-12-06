import numpy as np
import plotly.graph_objs as go
from scipy.spatial.transform import Rotation as R

def compute_vector(roll, pitch, yaw):
    # unit vector in z direction
    initial_vector = np.array([0, 0, 1])
    # get rotation from euler angles
    rotation = R.from_euler('xyz', np.array([roll, pitch, yaw]))
    # rotate vector
    rotated_vector = rotation.apply(initial_vector)
    return rotated_vector

def generate_plot(roll,pitch,yaw, step):
# compute all vector endpoints
    vectors = np.array([compute_vector(r, p, y)for r, p, y in zip(roll, pitch, yaw)])
    x_end, y_end, z_end = vectors[:, 0], vectors[:, 1], vectors[:, 2]


    fig = go.Figure()

    fig.add_trace(go.Scatter3d(
        x=[0, x_end[0]],
        y=[0, y_end[0]],
        z=[0, z_end[0]],
        mode='lines',
        line=dict(color='blue', width=5),
        name="Vector"
    ))

    fig.add_trace(go.Cone(
        x=[x_end[0]],
        y=[y_end[0]],
        z=[z_end[0]],
        u=[x_end[0]],
        v=[y_end[0]],
        w=[z_end[0]],
        sizemode="absolute",
        sizeref=0.3,
        anchor="tip",
        colorscale=[[0, "blue"], [1, "blue"]],
        showscale=False,
        name="Arrowhead"
    ))

    # Add frames for animation
    frames = [
        go.Frame(
            data=[
                go.Scatter3d(
                    x=[0, x_end[k]],
                    y=[0, y_end[k]],
                    z=[0, z_end[k]],
                    mode='lines',
                    line=dict(color='blue', width=5),
                ),
                go.Cone(
                    x=[x_end[k]],
                    y=[y_end[k]],
                    z=[z_end[k]],
                    u=[x_end[k]],
                    v=[y_end[k]],
                    w=[z_end[k]],
                    sizemode="absolute",
                    sizeref=0.3,
                    anchor="tip",
                    colorscale=[[0, "blue"], [1, "blue"]],
                    showscale=False,
                )
            ],
            name=f"frame{k}"
        )
        for k in range(0, len(roll), step)
    ]

    fig.frames = frames

    # Add sliders for interactivity
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[-1, 1], title="X"),
            yaxis=dict(range=[-1, 1], title="Y"),
            zaxis=dict(range=[-1, 1], title="Z"),
        ),
        updatemenus=[
            {
                'type': 'buttons',
                'showactive': False,
                'buttons': [
                    {
                        'label': 'Play',
                        'method': 'animate',
                        'args': [None, dict(frame=dict(duration=10, redraw=True), fromcurrent=True)],
                    },
                    {
                        'label': 'Pause',
                        'method': 'animate',
                        'args': [[None], dict(frame=dict(duration=0, redraw=False), mode='immediate')],
                    },
                ],
            }
        ],
        sliders=[{
            'steps': [
                {'args': [[f"frame{k}"], dict(mode='immediate', frame=dict(duration=10, redraw=True))],
                'method': 'animate'} for k in range(0, len(roll), step)
            ],
            'currentvalue': {'prefix': 'Time: ', 'font': {'size': 20}},
            'pad': {'t': 50},
        }]
    )

    fig.show()