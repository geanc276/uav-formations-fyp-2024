import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State, ALL, MATCH
import plotly.graph_objects as go
import json
import os
import shutil
import math
import random
import webbrowser
from threading import Timer

# Create the Dash app
app = dash.Dash(__name__)

app.layout = html.Div([
    dcc.Graph(id='3d-scatter-plot', clickData=None),
    html.Div([
        html.Button('Add Node', id='add-node-button', n_clicks=0),
        html.Button('Create Polygon', id='create-polygon-button', n_clicks=0),
        dcc.Input(id='polygon-points-input', type='number', placeholder='Number of points', min=3),
        dcc.Input(id='polygon-radius-input', type='number', placeholder='Radius', min=0.1, step=0.1),
        dcc.Input(id='polygon-height-input', type='number', placeholder='Height', step=0.1),
        html.Button('Save', id='save-button', n_clicks=0),
        dcc.Input(id='shape-name-input', type='text', placeholder='Enter shape name'),
        html.Label('Grid Scale:'),
        dcc.Slider(
            id='grid_scale-slider',
            min=1, max=10, step=1, value=3,
            marks={i: str(i) for i in range(1, 11)}
        ),
        html.Div(id='sliders-container')
    ]),
    dcc.Store(id='points-storage', data=[]),
    dcc.Store(id='selected-node-id', data=None)
])

def save_node_config(node, folder):
    config = {
        "id": node['id'],
        "takeoff_height": 4,
        "alt_constraint": {
            "min_altitude": 6,
            "height_displacement": node['z'],
            "gains": [2.4, 0.1, 0.2]
        },
        "pos_constraint": {
            "distance": math.sqrt(node['x']**2 + node['y']**2),
            "angle": math.degrees(math.atan2(node['y'], node['x'])),
            "gains": [3.5, 0.5, 0]
        },
        "heading_constraint": {
            "angle": 0,
            "gains": [0.2, 0.01, 0]
        }
    }
    
    filename = os.path.join(folder, f"drone_{node['id']}_config.json")
    with open(filename, 'w') as file:
        json.dump(config, file, indent=2)

@app.callback(
    [Output('points-storage', 'data'),
     Output('selected-node-id', 'data')],
    [Input('add-node-button', 'n_clicks'),
     Input('create-polygon-button', 'n_clicks'),
     Input({'type': 'slider', 'axis': 'x', 'id': ALL}, 'value'),
     Input({'type': 'slider', 'axis': 'y', 'id': ALL}, 'value'),
     Input({'type': 'slider', 'axis': 'z', 'id': ALL}, 'value'),
     Input({'type': 'input', 'axis': 'x', 'id': ALL}, 'value'),
     Input({'type': 'input', 'axis': 'y', 'id': ALL}, 'value'),
     Input({'type': 'input', 'axis': 'z', 'id': ALL}, 'value')],
    [State('polygon-points-input', 'value'),
     State('polygon-radius-input', 'value'),
     State('polygon-height-input', 'value'),
     State('points-storage', 'data'),
     State('selected-node-id', 'data')]
)
def manage_points(n_clicks_add, n_clicks_polygon, x_slider_values, y_slider_values, z_slider_values, x_input_values, y_input_values, z_input_values, num_points, radius, height, points, selected_node_id):
    ctx = dash.callback_context

    if not ctx.triggered:
        return points, selected_node_id

    triggered_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if 'add-node-button' in triggered_id:
        if n_clicks_add > len(points):
            color = f"#{random.randint(0, 0xFFFFFF):06x}"
            new_node = {'id': len(points) + 1, 'x': 0, 'y': 0, 'z': 0, 'color': color}
            points.append(new_node)
            selected_node_id = new_node['id']
    elif 'create-polygon-button' in triggered_id and num_points and radius:
        angle_step = 2 * math.pi / num_points
        for i in range(num_points):
            angle = i * angle_step
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = height if height is not None else 0
            color = f"#{random.randint(0, 0xFFFFFF):06x}"
            new_node = {'id': len(points) + 1, 'x': x, 'y': y, 'z': z, 'color': color}
            points.append(new_node)
        selected_node_id = points[-1]['id']
    else:
        for point in points:
            for prop in ctx.triggered:
                prop_id = json.loads(prop['prop_id'].split('.')[0])
                if point['id'] == prop_id['id']:
                    if prop_id['axis'] == 'x':
                        point['x'] = prop['value']
                    elif prop_id['axis'] == 'y':
                        point['y'] = prop['value']
                    elif prop_id['axis'] == 'z':
                        point['z'] = prop['value']
    
    return points, selected_node_id

@app.callback(
    Output('sliders-container', 'children'),
    [Input('3d-scatter-plot', 'clickData'),
     Input('selected-node-id', 'data'),
     Input('grid_scale-slider', 'value')],
    State('points-storage', 'data')
)
def update_sliders(clickData, selected_node_id, grid_scale, points):
    step = 0.001
    if points:
        if clickData:
            clicked_point_id = clickData['points'][0]['pointNumber']
            selected_point = points[clicked_point_id]
        elif selected_node_id:
            selected_point = next((p for p in points if p['id'] == selected_node_id), None)
        else:
            selected_point = None
            
        # marks = {i / 2: str(i / 2) for i in range(-6, 7)}  # from -3 to 3 with 0.5 steps
        marks = {i: str(i) for i in [x * 0.5 for x in range(-grid_scale * 2, grid_scale * 2 + 1)]}
        
        sliders = html.Div([
            html.Label(f"Node {selected_point['id']}"),
            html.Label('X:'),
            dcc.Slider(
                id={'type': 'slider', 'axis': 'x', 'id': selected_point['id']},
                min=-grid_scale, max=grid_scale, step=step, value=selected_point['x'],
                marks=marks  # Display marks for every half-integer and integer
            ),
            dcc.Input(
                id={'type': 'input', 'axis': 'x', 'id': selected_point['id']},
                type='number', min=-grid_scale, max=grid_scale, step=step, value=selected_point['x']
            ),
            html.Label('Y:'),
            dcc.Slider(
                id={'type': 'slider', 'axis': 'y', 'id': selected_point['id']},
                min=-grid_scale, max=grid_scale, step=step, value=selected_point['y'],
                marks=marks  # Display marks for every half-integer and integer
            ),
            dcc.Input(
                id={'type': 'input', 'axis': 'y', 'id': selected_point['id']},
                type='number', min=-grid_scale, max=grid_scale, step=step, value=selected_point['y']
            ),
            html.Label('Z:'),
            dcc.Slider(
                id={'type': 'slider', 'axis': 'z', 'id': selected_point['id']},
                min=-grid_scale, max=grid_scale, step=step, value=selected_point['z'],
                marks=marks  # Display marks for every half-integer and integer
            ),
            dcc.Input(
                id={'type': 'input', 'axis': 'z', 'id': selected_point['id']},
                type='number', min=-grid_scale, max=grid_scale, step=step, value=selected_point['z']
            )
        ])
        return [sliders]
    return []

@app.callback(
    [Output({'type': 'slider', 'axis': MATCH, 'id': MATCH}, 'value'),
     Output({'type': 'input', 'axis': MATCH, 'id': MATCH}, 'value')],
    [Input({'type': 'slider', 'axis': MATCH, 'id': MATCH}, 'value'),
     Input({'type': 'input', 'axis': MATCH, 'id': MATCH}, 'value')],
    [State({'type': 'slider', 'axis': MATCH, 'id': MATCH}, 'value'),
     State({'type': 'input', 'axis': MATCH, 'id': MATCH}, 'value')]
)
def sync_slider_and_input(slider_value, input_value, current_slider_value, current_input_value):
    ctx = dash.callback_context

    if not ctx.triggered:
        return current_slider_value, current_input_value

    triggered_id = ctx.triggered[0]['prop_id']

    if 'slider' in triggered_id:
        return slider_value, slider_value
    elif 'input' in triggered_id:
        return input_value, input_value

    return current_slider_value, current_input_value

@app.callback(
    Output('3d-scatter-plot', 'figure'),
    [Input('points-storage', 'data'),
     Input('grid_scale-slider', 'value')]
)
def update_plot(points, grid_scale):
    scatter_points = go.Scatter3d(
        x=[point['x'] for point in points],
        y=[point['y'] for point in points],
        z=[point['z'] for point in points],
        mode='markers',
        marker=dict(
            size=4,
            color=[point['color'] for point in points],
        )
    )

    origin_point = go.Scatter3d(
        x=[0],
        y=[0],
        z=[0],
        mode='markers',
        marker=dict(
            size=6,
            color='red',
        )
    )

    layout = go.Layout(
        scene=dict(
            xaxis=dict(title='X', range=[-grid_scale, grid_scale]),
            yaxis=dict(title='Y', range=[-grid_scale, grid_scale]),
            zaxis=dict(title='Z', range=[-grid_scale, grid_scale]),
            aspectmode='cube'
        )
    )

    fig = go.Figure(data=[scatter_points, origin_point], layout=layout)
    return fig

@app.callback(
    Output('shape-name-input', 'value'),
    Input('save-button', 'n_clicks'),
    State('shape-name-input', 'value'),
    State('points-storage', 'data')
)
def save_shape(n_clicks, shape_name, points):
    if n_clicks > 0 and shape_name:
        folder = f"configs/{shape_name}_config"
        if os.path.exists(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)
        for point in points:
            save_node_config(point, folder)
        return ''
    return shape_name

def open_browser():
    webbrowser.open_new_tab("http://127.0.0.1:8050/")

if __name__ == "__main__":
    Timer(1, open_browser).start()  # Open the browser after a short delay
    app.run_server(debug=True)
