import json
import plotly.graph_objs as go
import numpy as np


def get_traces(data: dict, x_axis, skip_index):

    colors = [
        "rgb(0, 0, 0)", "rgb(255, 0, 0)", "rgb(0, 102, 204)", "rgb(0, 153, 0)", "rgb(204, 0, 204)",
        "rgb(255, 128, 0)", "rgb(128, 0, 255)", "rgb(128, 128, 128)", "rgb(0, 153, 153)", "rgb(255, 204, 0)",
        "rgb(76, 0, 153)", "rgb(0, 0, 255)", "rgb(0, 255, 0)", "rgb(255, 0, 255)", "rgb(255, 102, 102)",
        "rgb(153, 102, 255)", "rgb(255, 255, 0)", "rgb(0, 204, 102)", "rgb(102, 51, 153)", "rgb(255, 51, 0)",
        "rgb(0, 51, 102)", "rgb(102, 204, 0)", "rgb(255, 153, 51)", "rgb(153, 0, 0)", "rgb(51, 255, 255)"
    ]

    traces = []

    aoi = go.Scattergl(
        x=x_axis,
        y=(np.asarray(data["ego"]["aoi"][skip_index:]) / 1000_000),
        mode="lines+markers",
        name="aoi [ms]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(aoi)

    trace_tactical_speed = go.Scattergl(
        x=x_axis,
        y=data["ego"]["ego_tactical_speed"][skip_index:],
        mode="lines+markers",
        name="tactical_speed [m/s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_tactical_speed)

    trace_ego_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_pos"][skip_index:],
        mode="lines+markers",
        name="ego_pos",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pos)

    trace_ego_pred_go_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_pred_go_pos"][skip_index:],
        mode="lines+markers",
        name="ego_pred_go_pos",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pred_go_pos)

    ego_d_to_cr = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_d_to_cr"][skip_index:],
        mode="lines+markers",
        name="ego_d_to_cr [m]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(ego_d_to_cr)

    ego_ttcr = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_ttcr"][skip_index:],
        mode="lines+markers",
        name="ego_ttcr [s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(ego_ttcr)

    trace_target_acc = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_acc"][skip_index:],
        mode="lines+markers",
        name="target_acc [m/s2]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_acc)


    trace_target_ttcr = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_ttcr"][skip_index:],
        mode="lines+markers",
        name="target_ttcr [s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_ttcr)

    trace_target_d_to_cr = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_d_to_cr"][skip_index:],
        mode="lines+markers",
        name="target_d_to_cr [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_to_cr)

    trace_target_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_pos"][skip_index:],
        mode="lines+markers",
        name="target_pos",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_pos)

    trace_target_d_front = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_d_front"][skip_index:],
        mode="lines+markers",
        name="target_d_front [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_front)

    trace_target_d_front = go.Scatter(
        x=x_axis,
        y=data["ego"]["aoi_seconds"][skip_index:],
        mode="lines+markers",
        name="aoi_seconds [s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_front)

    trace_ego_d_front = go.Scatter(
        x=x_axis,
        y=data["ego"]["kalman_acc"][skip_index:],
        mode="lines+markers",
        name="kalman_acc [m/s2]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_d_front)

    trace_ego_v = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_front_d"][skip_index:],
        mode="lines+markers",
        name="ego_front_d [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_v)

    trace_msg_id = go.Scatter(
        x=x_axis,
        y=data["ego"]["msg_id"],
        mode="lines+markers",
        name="msg_id",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_msg_id)

    trace_msg_time = go.Scatter(
        x=x_axis,
        y=(np.asarray(data["ego"]["msg_time"]) - data["ego"]["msg_time"][0]) /1000_000_000,
        mode="lines+markers",
        name="msg_time",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_msg_time)

    return traces



def plot(file_name):
    with open(file_name, "r") as f:
        parsed = json.load(f)

    # Create the graph layout
    plot_heading = ("Debug chart {0}, cause {1}".format(parsed["exp_name"], parsed["term_cause"]))

    dec_time = [x for x in parsed["ego"]["decision_time"] if x>-1]
    skip_index = len(parsed["ego"]["decision_time"]) - len(dec_time)
    x_axis = (np.asarray(dec_time) - dec_time[0])/1000_000
    traces = get_traces(parsed, x_axis, skip_index)

    # Create the graph layout
    layout = go.Layout(
        title=plot_heading,
        xaxis=dict(range=[x_axis[0], x_axis[-1]]),
        yaxis=dict(range=[-5, 35]),
        hovermode='x unified'
    )

    fig = go.Figure(data=traces, layout=layout)
    fig.show()


if __name__ == '__main__':

    fileName = "/home/gianfi/KvaserCar_outdoor_test/data/1747663167.0827518.json"
    plot(fileName)