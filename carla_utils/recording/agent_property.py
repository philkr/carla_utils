class AgentProperty(dict):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def __getattr__(self, item):
        if item in self:
            return self[item]
        super().__getattr__(item)

    @classmethod
    def get(cls, name):
        if name in _prop:
            return _prop[name]
        return None


def Vector3D(x=0, y=0, z=0):
    import numpy as np
    return np.array([x, y, z])


_prop = {}
# AUTO GENERATED BELOW (DO NOT MODIFY)
_prop['walker.pedestrian.0026'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0025'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0023'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0022'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0024'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0020'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0019'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0018'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0017'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0016'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0014'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.650000))
_prop['walker.pedestrian.0013'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.650000))
_prop['walker.pedestrian.0012'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.650000))
_prop['walker.pedestrian.0010'] = AgentProperty(extent=Vector3D(x=0.250000, y=0.250000, z=0.550000))
_prop['walker.pedestrian.0009'] = AgentProperty(extent=Vector3D(x=0.250000, y=0.250000, z=0.550000))
_prop['walker.pedestrian.0008'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0006'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0005'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0004'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0003'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0002'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0001'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['walker.pedestrian.0007'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['vehicle.mustang.mustang'] = AgentProperty(extent=Vector3D(x=2.358763, y=0.947413, z=0.650470))
_prop['vehicle.lincoln.mkz2017'] = AgentProperty(extent=Vector3D(x=2.450842, y=1.064162, z=0.755373))
_prop['walker.pedestrian.0011'] = AgentProperty(extent=Vector3D(x=0.250000, y=0.250000, z=0.550000))
_prop['vehicle.volkswagen.t2'] = AgentProperty(extent=Vector3D(x=2.240218, y=1.034658, z=1.018896))
_prop['walker.pedestrian.0015'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['vehicle.tesla.model3'] = AgentProperty(extent=Vector3D(x=2.395890, y=1.081725, z=0.744160))
_prop['vehicle.tesla.cybertruck'] = AgentProperty(extent=Vector3D(x=3.136776, y=1.194787, z=1.049096))
_prop['vehicle.audi.etron'] = AgentProperty(extent=Vector3D(x=2.427854, y=1.016378, z=0.824680))
_prop['vehicle.diamondback.century'] = AgentProperty(extent=Vector3D(x=0.821422, y=0.186258, z=0.511951))
_prop['vehicle.bh.crossbike'] = AgentProperty(extent=Vector3D(x=0.743644, y=0.429629, z=0.539789))
_prop['vehicle.kawasaki.ninja'] = AgentProperty(extent=Vector3D(x=1.016676, y=0.401290, z=0.572727))
_prop['walker.pedestrian.0021'] = AgentProperty(extent=Vector3D(x=0.340000, y=0.340000, z=0.930000))
_prop['vehicle.yamaha.yzf'] = AgentProperty(extent=Vector3D(x=1.104723, y=0.433517, z=0.625573))
_prop['vehicle.harley-davidson.low_rider'] = AgentProperty(extent=Vector3D(x=1.177870, y=0.381839, z=0.638285))
_prop['vehicle.toyota.prius'] = AgentProperty(extent=Vector3D(x=2.256761, y=1.003407, z=0.762417))
_prop['vehicle.seat.leon'] = AgentProperty(extent=Vector3D(x=2.096415, y=0.908093, z=0.736916))
_prop['vehicle.nissan.patrol'] = AgentProperty(extent=Vector3D(x=2.302255, y=0.965797, z=0.927423))
_prop['vehicle.mini.cooperst'] = AgentProperty(extent=Vector3D(x=1.902900, y=0.985138, z=0.737515))
_prop['vehicle.mercedes-benz.coupe'] = AgentProperty(extent=Vector3D(x=2.513388, y=1.075773, z=0.817764))
_prop['vehicle.jeep.wrangler_rubicon'] = AgentProperty(extent=Vector3D(x=1.933110, y=0.952598, z=0.938968))
_prop['vehicle.dodge_charger.police'] = AgentProperty(extent=Vector3D(x=2.487122, y=1.019201, z=0.777148))
_prop['vehicle.gazelle.omafiets'] = AgentProperty(extent=Vector3D(x=0.917720, y=0.164464, z=0.562829))
_prop['vehicle.citroen.c3'] = AgentProperty(extent=Vector3D(x=1.993842, y=0.925424, z=0.808555))
_prop['vehicle.chevrolet.impala'] = AgentProperty(extent=Vector3D(x=2.678740, y=1.016602, z=0.705329))
_prop['vehicle.nissan.micra'] = AgentProperty(extent=Vector3D(x=1.816688, y=0.922557, z=0.750732))
_prop['vehicle.carlamotors.carlacola'] = AgentProperty(extent=Vector3D(x=2.601919, y=1.307286, z=1.233722))
_prop['vehicle.bmw.isetta'] = AgentProperty(extent=Vector3D(x=1.103648, y=0.740460, z=0.689374))
_prop['vehicle.bmw.grandtourer'] = AgentProperty(extent=Vector3D(x=2.305503, y=1.120857, z=0.833638))
_prop['vehicle.audi.tt'] = AgentProperty(extent=Vector3D(x=2.090605, y=0.997059, z=0.692648))
_prop['vehicle.audi.a2'] = AgentProperty(extent=Vector3D(x=1.852685, y=0.894339, z=0.773544))
