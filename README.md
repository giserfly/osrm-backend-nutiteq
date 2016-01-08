## About

This is Nutiteq-specific version of OSRM project. It keeps original OSRM extract and prepare stages
but replaces online routing engine with Nutiteq routing engine based on offline packages, prepared with Nutiteq osrm-converter (separate project).

The Open Source Routing Machine is a high performance routing engine written in C++11 designed to run on OpenStreetMap data.

## Building

For instructions on how to [build](https://github.com/Project-OSRM/osrm-backend/wiki/Building-OSRM) and [run OSRM](https://github.com/Project-OSRM/osrm-backend/wiki/Running-OSRM), please consult [the Wiki](https://github.com/Project-OSRM/osrm-backend/wiki).

To quickly try OSRM use our [free and daily updated online service](http://map.project-osrm.org)

## Documentation

See the Wiki's [server API documentation](https://github.com/Project-OSRM/osrm-backend/wiki/Server-api) as well as the [library API documentation](https://github.com/Project-OSRM/osrm-backend/wiki/Library-api)

## References in publications

When using the code in a (scientific) publication, please cite

```
@inproceedings{luxen-vetter-2011,
 author = {Luxen, Dennis and Vetter, Christian},
 title = {Real-time routing with OpenStreetMap data},
 booktitle = {Proceedings of the 19th ACM SIGSPATIAL International Conference on Advances in Geographic Information Systems},
 series = {GIS '11},
 year = {2011},
 isbn = {978-1-4503-1031-4},
 location = {Chicago, Illinois},
 pages = {513--516},
 numpages = {4},
 url = {http://doi.acm.org/10.1145/2093973.2094062},
 doi = {10.1145/2093973.2094062},
 acmid = {2094062},
 publisher = {ACM},
 address = {New York, NY, USA},
}
```

