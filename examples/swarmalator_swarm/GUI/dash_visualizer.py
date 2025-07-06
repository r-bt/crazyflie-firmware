import dash
from dash import dcc, html
import plotly.graph_objects as go
from dash.dependencies import Input, Output
import threading
import webbrowser

class DashVisualizer:
    def __init__(self):
        self.app = dash.Dash(__name__)
        self.positions = []
        
        # Create initial figure with uirevision
        self.fig = go.Figure(
            data=[go.Scatter3d(
                x=[0], y=[0], z=[0],
                mode='markers', 
                marker=dict(size=10, color='red')
            )],
            layout=go.Layout(
                scene=dict(
                    xaxis=dict(range=[-5, 5], title='X Position'),
                    yaxis=dict(range=[-5, 5], title='Y Position'),
                    zaxis=dict(range=[-5, 5], title='Z Position'),
                    aspectmode='cube',
                    camera=dict(
                        up=dict(x=0, y=0, z=1),
                        center=dict(x=0, y=0, z=0),
                        eye=dict(x=1.5, y=1.5, z=1.5)
                    ),
                ),
                title="Crazyflie Swarm 3D Visualization",
                uirevision=True  # This preserves UI state
            )
        )

        # Dash layout
        self.app.layout = html.Div([
            dcc.Graph(id='3d-plot', figure=self.fig),
            dcc.Interval(
                id='interval-component',
                interval=100,
                n_intervals=0
            )
        ])

        @self.app.callback(
            Output('3d-plot', 'figure'),
            Input('interval-component', 'n_intervals')
        )
        def update_plot(_):
            if not self.positions:
                return self.fig

            valid_positions = [p for p in self.positions if p is not None]
            if not valid_positions:
                return self.fig

            # Update only the data, preserving the layout
            fig_copy = go.Figure(self.fig)
            fig_copy.data = []
            fig_copy.add_trace(
                go.Scatter3d(
                    x=[p['x'] for p in valid_positions],
                    y=[p['y'] for p in valid_positions],
                    z=[p['z'] for p in valid_positions],
                    mode='markers',
                    marker=dict(size=10, color='red')
                )
            )
            return fig_copy

    def update_positions(self, positions):
        self.positions = positions

    def start(self, port=8050):
        def open_browser():
            webbrowser.open(f'http://127.0.0.1:{port}/')
        
        threading.Timer(1.5, open_browser).start()
        self.app.run(debug=False, port=port)

    def run_in_thread(self, port=8050):
        self.server_thread = threading.Thread(
            target=self.start,
            args=(port,),
            daemon=True
        )
        self.server_thread.start()