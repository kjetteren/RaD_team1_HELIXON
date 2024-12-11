import numpy as np
import plotly.graph_objs as go
from scipy.spatial.transform import Rotation as R
from collections import deque
import time
import streamlit as st

import socket
import struct

def quaternion_to_euler_angles(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    Quaternion format assumed as [w, x, y, z].
    """
    w, x, y, z = q
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.pi / 2 * np.sign(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

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

    return fig



def generate_plot_realtime():
    Fig = st.empty()
    
    # Initialize Plotly figure
    fig = go.Figure()
    # fig.update_layout(
    #     autosize=False,
    #     width=800,
    #     height=700,
    #     margin=dict(l=0, r=0, b=0, t=0),
    #     scene=dict(
    #         xaxis=dict(range=[-1, 1], autorange=False, tickvals=[-1, -0.5, 0, 0.5, 1], showspikes=False),
    #         yaxis=dict(range=[-1, 1], autorange=False, tickvals=[-1, -0.5, 0, 0.5, 1], showspikes=False),
    #         zaxis=dict(range=[-1, 1], autorange=False, tickvals=[-1, -0.5, 0, 0.5, 1], showspikes=False),
    #         aspectmode='cube',
    #         camera=dict(
    #             up=dict(x=0, y=0, z=.5),
    #             center=dict(x=0, y=0, z=0),
    #             eye=dict(x=1.25, y=1.25, z=1.25)
    #         )
    #     ),
    # )

    UDP_IP = "0.0.0.0"  # Replace with your desired IP
    UDP_PORT = 12345    # Replace with your port number
    BUFFER_SIZE = 76    # 19 floats * 4 bytes per float = 76 bytes

    rolls = deque(maxlen=100)
    pitches = deque(maxlen=100)
    yaws = deque(maxlen=100)

    # Sending Socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start_message = "0"
    send_sock.sendto(start_message.encode(), ("192.168.4.1", UDP_PORT))

    # Receiving Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    while True:
        # Process data
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)  # Receive data from the sensor
            float_data = struct.unpack('f' * 19, data)  # Unpack 19 floats from the data
            quaternion = float_data[12:16] 
            roll, pitch, yaw = quaternion_to_euler_angles(quaternion)

            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)

            x,y,z = compute_vector(roll, pitch, yaw)

        except Exception as e:
            print(f"Error: {e}")
            continue

        fig.data = []
        fig.add_trace(go.Scatter3d(x=[0, x], y=[0, y], z=[0, z], mode='lines', line=dict(width=10, color='red'), hoverinfo='none'))
        fig.add_trace(go.Cone(x=[0.9 * x], y=[0.9 * y], z=[0.9 * z], u=[0.1 * x], v=[0.1 * y], w=[0.1 * z], sizemode="scaled", sizeref=2.5, showscale=False, colorscale=[(0, 'red'), (1, 'red')], cmin=-1, cmax=1, hoverinfo='none'))

        with Fig.container():
            st.plotly_chart(fig, use_container_width=True)
        time.sleep(1)

def generate_plot_realtime_lesko():
    # Reserve space for the plot
    Fig = st.empty()

    # Initialize Plotly figure outside the loop
    fig = go.Figure()
    fig.update_layout(
        autosize=False,
        width=800,
        height=700,
        margin=dict(l=0, r=0, b=0, t=0),
        scene=dict(
            xaxis=dict(range=[-1, 1], autorange=False, tickvals=[-1, -0.5, 0, 0.5, 1], showspikes=False),
            yaxis=dict(range=[-1, 1], autorange=False, tickvals=[-1, -0.5, 0, 0.5, 1], showspikes=False),
            zaxis=dict(range=[-1, 1], autorange=False, tickvals=[-1, -0.5, 0, 0.5, 1], showspikes=False),
            aspectmode='cube',
            camera=dict(
                up=dict(x=0, y=0, z=.5),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=1.25, y=1.25, z=1.25)
            )
        ),
    )

    # Initialize data lists
    rolls, pitches, yaws = deque(maxlen=100), deque(maxlen=100), deque(maxlen=100)

    # Network setup
    UDP_IP = "0.0.0.0"  # Replace with your desired IP
    UDP_PORT = 12345    # Replace with your port number
    BUFFER_SIZE = 76    # 19 floats * 4 bytes per float = 76 bytes
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    # Main loop
    while True:
        try:
            # Read data
            data, addr = sock.recvfrom(BUFFER_SIZE)
            float_data = struct.unpack('f' * 19, data)
            quaternion = float_data[12:16] 
            roll, pitch, yaw = quaternion_to_euler_angles(quaternion)

            # Append latest data
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)

            # Compute rotation vector
            x, y, z = compute_vector(roll, pitch, yaw)

            # Update Plotly data
            fig.data = []  # Clear previous traces
            fig.add_trace(go.Scatter3d(
                x=[0, x],
                y=[0, y],
                z=[0, z],
                mode='lines',
                line=dict(width=10, color='red'),
                hoverinfo='none'
            ))
            fig.add_trace(go.Cone(
                x=[0.9 * x],
                y=[0.9 * y],
                z=[0.9 * z],
                u=[0.1 * x],
                v=[0.1 * y],
                w=[0.1 * z],
                sizemode="scaled",
                sizeref=2.5,
                showscale=False,
                colorscale=[(0, 'red'), (1, 'red')],
                cmin=-1,
                cmax=1,
                hoverinfo='none'
            ))

            # Update the placeholder with the latest figure
            with Fig.container():
                st.plotly_chart(fig, use_container_width=True, config={'displayModeBar': False})

        except Exception as e:
            print(f"Error: {e}")
            continue

        # Introduce a small delay to prevent rapid updates
        time.sleep(0.1)

generate_plot_realtime()
