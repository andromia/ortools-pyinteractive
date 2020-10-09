from typing import List
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd


def visualize_clusters(
    dest_lats: List[float], dest_lons: List[float], clusters: List[int]
):
    cluster_strs = [str(c) for c in clusters]
    data = pd.DataFrame(
        data={"lats": dest_lats, "lons": dest_lons, "clusters": cluster_strs}
    )
    data.clusters = data.clusters.astype(str)

    fig = px.scatter_geo(
        data, lat="lats", lon="lons", color="clusters", template="plotly_dark"
    )
    fig.show()


def visualize_solution(vehicles: "Vehicles") -> None:
    # base
    lats = []
    lons = []
    text = []

    # lines
    lat_paths = []
    lon_paths = []
    for i, r in enumerate(vehicles):
        for j, v in enumerate(r):
            lats.append(v.lat)
            lons.append(v.lon)
            text.append(f"demand: {v.demand}")

            if j < len(r) - 1:
                lat_paths.append([v.lat, r[j + 1].lat])
                lon_paths.append([v.lon, r[j + 1].lon])

    fig = go.Figure()

    for i in range(len(lat_paths)):
        fig.add_trace(
            go.Scattergeo(
                locationmode="USA-states",
                lat=[lat_paths[i][0], lat_paths[i][1]],
                lon=[lon_paths[i][0], lon_paths[i][1]],
                mode="lines",
                line=dict(width=1, color="red"),
                # opacity = float(df_flight_paths['cnt'][i]) / float(df_flight_paths['cnt'].max()),
            )
        )

    fig.add_trace(
        go.Scattergeo(
            locationmode="USA-states",
            lat=lats,
            lon=lons,
            hovertext=text,
            hoverinfo="text",
            mode="markers",
            marker=dict(
                size=5,
                color="rgb(255, 0, 0)",
                line=dict(width=3, color="rgba(68, 68, 68, 0)"),
            ),
        )
    )

    fig.update_layout(
        title_text="",
        showlegend=False,
        template="plotly_dark",
        geo=dict(
            scope="north america",
            projection_type="azimuthal equal area",
            showland=True,
            landcolor="rgb(243, 243, 243)",
            countrycolor="rgb(204, 204, 204)",
        ),
    )

    fig.show()
