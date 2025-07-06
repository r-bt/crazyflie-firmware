import plotly.graph_objects as go
import numpy as np

# Create an array of points for the animation
frames = []
n_frames = 100
t = np.linspace(0, 2 * np.pi, n_frames)
x_vals = 2 * np.cos(t)  # Circle movement for X
y_vals = 2 * np.sin(t)  # Circle movement for Y
z_vals = np.zeros(n_frames)  # Constant Z

# Create initial scatter plot
fig = go.Figure(
    data=[
        go.Scatter3d(
            x=[x_vals[0]],
            y=[y_vals[0]],
            z=[z_vals[0]],
            mode="markers",
            marker=dict(size=10, color="red"),
        )
    ],
    layout=go.Layout(
        scene=dict(
            xaxis=dict(range=[-3, 3]),
            yaxis=dict(range=[-3, 3]),
            zaxis=dict(range=[-1, 1]),
        ),
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
                buttons=[
                    dict(
                        label="Play",
                        method="animate",
                        args=[
                            None,
                            dict(
                                frame=dict(duration=50, redraw=True), fromcurrent=True
                            ),
                        ],
                    )
                ],
            )
        ],
    ),
    frames=[
        go.Frame(
            data=[
                go.Scatter3d(
                    x=[x],
                    y=[y],
                    z=[z],
                    mode="markers",
                    marker=dict(size=10, color="red"),
                )
            ],
            name=str(i),
        )
        for i, (x, y, z) in enumerate(zip(x_vals, y_vals, z_vals))
    ],
)

# Show the plot
fig.show()
