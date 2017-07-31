function CenterMapXY(X, Y) {
    map.getView().setCenter([X, Y]);
    map.getView().setZoom(10);
}

var points = [];

var markerLayer = new ol.layer.Vector({
    source: new ol.source.Vector({features: [], projection: 'EPSG:4326'})
});

var map = new ol.Map({
    view: new ol.View({center: [0, 0], zoom: 12}),
    layers: [new ol.layer.Tile({source: new ol.source.OSM()}),
        markerLayer], // basic OSM layer
    target: 'map'
});

var center = ol.proj.transform([30.1037, 59.8145], 'EPSG:4326', 'EPSG:3857');
CenterMapXY(center[0], center[1]);

function addMarkerXY(x, y) {
    var geom = new ol.geom.Point([x, y]);
    var feature = new ol.Feature(geom);
    feature.setStyle([
        new ol.style.Style({
            image: new ol.style.Icon(({
                anchor: [0.5, 1],
                anchorXUnits: 'fraction',
                anchorYUnits: 'fraction',
                opacity: 1,
                src: 'https://openlayers.org/en/v3.19.1/examples/data/icon.png'
            }))
        })
    ]);

    map.getLayers().item(1).getSource().addFeature(feature);
}

map.on('click', function (evt) {
    var xy = evt.coordinate; // projection!!

    if (points.length >= 2) {
        points = [];
        markerLayer.getSource().clear();
        map.getLayers().removeAt(3);
        map.getLayers().removeAt(2);
    }

    points.push(ol.proj.transform(xy, 'EPSG:3857', 'EPSG:4326'));
    addMarkerXY(xy[0], xy[1]);

    if (map.getLayers().item(1).getSource().getFeatures().length === 2) {

        xhttp = new XMLHttpRequest();

        xhttp.open("GET", "launcher.php?lat1=" + points[0][0]
            + "&lon1=" + points[0][1] + "&lat2=" + points[1][0]
            + "&lon2=" + points[1][1], true);

        xhttp.send();

        xget_points_cpu = new XMLHttpRequest();

        xget_points_cpu.onreadystatechange = function () {
            if (xget_points_cpu.readyState === 4) {
                points = [];
                show_route(xget_points_cpu.responseText, 0);
            }

            //todo: real-time time
        };

        xget_points_cpu.open("GET", "out/points_cpu.txt", true);
        xget_points_cpu.send();

        xget_points_gpu = new XMLHttpRequest();

        xget_points_gpu.onreadystatechange = function () {
            if (xget_points_gpu.readyState === 4) {
                points = [];
                show_route(xget_points_gpu.responseText, 1);
            }
        };

        xget_points_gpu.open("GET", "out/points_gpu.txt", true);
        xget_points_gpu.send();

    }
});

function show_route(contents, device) {
    var lines = contents.split("\n");
    var time = parseFloat(lines[1]);

    for (var i = 2; i < lines.length; i++) {
        var text_coords = lines[i].split(" ");
        if (text_coords.length === 2) {
            points.push(ol.proj.fromLonLat([parseFloat(text_coords[1]), parseFloat(text_coords[0])]));
        }
    }

    if (device === 0) {
        document.getElementById("cpu_time").innerHTML = time.toFixed(3).toString() * 1000.0 + "ms";
    } else if (device === 1) {
        document.getElementById("gpu_time").innerHTML = time.toFixed(3).toString() * 1000.0 + "ms";
    }

    var routeLayer;

    if (device === 0) {
        routeLayer = new ol.layer.Vector({
            source: new ol.source.Vector({
                features: [new ol.Feature({geometry: new ol.geom.LineString(points, 'XY')})]
            }),
            style: new ol.style.Style({
                fill: new ol.style.Fill({color: '#0091ea', weight: 8}),
                stroke: new ol.style.Stroke({color: '#0091ea', width: 4})
            }),
            name: 'route_cpu'
        });
    } else if (device === 1) {
        routeLayer = new ol.layer.Vector({
            source: new ol.source.Vector({
                features: [new ol.Feature({geometry: new ol.geom.LineString(points, 'XY')})]
            }),
            style: new ol.style.Style({
                fill: new ol.style.Fill({color: '#66cc00', weight: 8}),
                stroke: new ol.style.Stroke({color: '#66cc00', width: 4})
            }),
            name: 'route_gpu'
        });
    }

    map.addLayer(routeLayer);

    // Start and end of route
    if (device === 0) {
        markerLayer.getSource().clear();
        addMarkerXY(points[0][0], points[0][1]);
        addMarkerXY(points[points.length - 1][0], points[points.length - 1][1]);
    }

    // Center map on route and zoom in/out
    // var extent = markerLayer.getSource().getExtent();
    // if (markerLayer.getSource().getFeatures().length > 0) {
    //     map.getView().fit(extent, map.getSize());
    // }
}
