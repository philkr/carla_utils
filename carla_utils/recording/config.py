"""
  Configuration file management
  If you want to create a new configuration create a class with type decorators, and register it

    @Configuration.register('foo')
    class Foo(Settings):
        class Bar(Settings):
            name: str
            id: int = 0

        name: str = Required
        number: int
        bars: list[Bar]

  This will load the following yaml file
    ---
    foo:
        name: "hi"
        bars:
          - name: "mo"
            id: 1
          - name: "mi"
            id: 2

  You can register settings as repeated and provide multiple documents, or even include other documents.

    @Configuration.register('bar', repeated=True)
    class Bar(Settings):
        name: str = Required

    ---
    bar:
        name: "bar1"
    ---
    bar:
        name: "bar2"
    #include "some_other.yaml"
"""
from typing import get_type_hints, get_origin, get_args
from carla_utils.util import Printable

__all__ = ["Configuration", "Required", "Settings"]


def load_file_with_include(fn):
    import re
    with open(fn, 'r') as f:
        return re.sub('\#include +["<](.*)[">]', lambda m: load_file_with_include(fn.parent / m.group(1)), f.read())


class ForAll:
    def __init__(self, objects):
        self._objects = objects

    def __getitem__(self, item):
        return ForAll([o[item] for o in self._objects])

    def __setitem__(self, key, value):
        for o in self._objects:
            o[key] = value

    def __getattr__(self, item):
        if item[0] != '_':
            return ForAll([getattr(o, item) for o in self._objects])

    def __setattr__(self, name, value):
        if name[0] == '_':
            return super().__setattr__(name, value)
        else:
            for o in self._objects:
                setattr(o, name, value)


class EllipsisList(list):
    def __getitem__(self, item):
        if item is ...:
            return ForAll(list(self))
        else:
            return super().__getitem__(item)


class Configuration:
    _all = {}

    @staticmethod
    def register(name, repeated=False):
        def _wrapper(cls):
            Configuration._all[name] = (cls, repeated)
            cls.__str__ = Printable.__str__
            return cls
        return _wrapper

    @staticmethod
    def _find(name):
        return Configuration._all[name][0] if name in Configuration._all else None

    @staticmethod
    def _is_repeated(name):
        return Configuration._all[name][1] if name in Configuration._all else None

    @staticmethod
    def _has(name):
        return name in Configuration._all

    @staticmethod
    def from_file(*fns):
        from pathlib import Path
        settings = []
        for fn in fns:
            fn = Path(fn)
            if fn.name.endswith('.json'):
                import json
                with open(fn, 'r') as f:
                    settings.append(json.load(f))
            elif fn.name.endswith('.yaml'):
                import yaml
                settings.extend(list(yaml.load_all(load_file_with_include(fn), Loader=yaml.FullLoader)))
            else:
                raise ValueError('Unknown file extension for configuration "%s"' % fn.name)
        return Configuration.from_dict(*settings)

    @staticmethod
    def from_dict(*settings):
        r = Configuration()
        for o in settings:
            for k, v in o.items():
                tpe = Configuration._find(k)
                assert tpe is not None, 'No configuration found for {!r}'.format(k)
                if Configuration._is_repeated(k):
                    if hasattr(r, k):
                        getattr(r, k).append(tpe(**v))
                    else:
                        setattr(r, k, EllipsisList([tpe(**v)]))
                else:
                    assert not hasattr(r, k), 'Multiple configurations for {!r} found'.format(k)
                    setattr(r, k, tpe(**v))
        return r

    def validate(self):
        # Add default configs for the ones missing
        for k, (cls, repeated) in Configuration._all.items():
            if not hasattr(self, k):
                setattr(self, k, [] if repeated else cls())

        # Make sure all configs are complete
        for v in vars(self):
            if hasattr(v, 'validate'):
                v.validate()
            elif isinstance(v, list):
                for vv in v:
                    if hasattr(vv, 'validate'):
                        vv.validate()

    @staticmethod
    def add_argument(parser):
        parser.add_argument('--config', nargs='+', default=[])
        parser.add_argument('--config_override', nargs='+', default=[])

    @staticmethod
    def from_args(args):
        cfg = Configuration.from_file(*args.config)
        # TODO: override in a less hacky way
        for o in args.config_override:
            exec(o, {k: getattr(cfg, k) for k in vars(cfg)})
        cfg.validate()
        return cfg


class Required:
    pass


class Settings(Printable):
    def __init__(self, **kwargs):
        th = get_type_hints(self.__class__)
        for k, v in kwargs.items():
            if k in th:
                setattr(self, k, convert_to(v, th[k], type(self).__name__+'.'+k))
            assert hasattr(self, k), 'Unknown settings value {!s}.{!s}'.format(type(self).__name__, k)

    def validate(self):
        for k in vars(self.__class__):
            assert getattr(self, k) != Required, "Required configuration value {!r} not specified".format(k)


def convert_to(o, tpe, dname=''):
    # Make sure v is of decorated type tpe
    if tpe is None:
        return o
    if get_origin(tpe) == list:
        assert isinstance(o, list), 'Setting {!r} expected type {!r} got {!r} ({!r})'.format(dname, tpe, type(o), o)
        l_tpe, *_ = *get_args(tpe), None
        return [convert_to(i, l_tpe, dname) for i in o]
    if get_origin(tpe) == dict:
        assert isinstance(o, dict), 'Setting {!r} expected type {!r} got {!r} ({!r})'.format(dname, tpe, type(o), o)
        k_tpe, v_tpe, *_ = *get_args(tpe), None, None
        return {convert_to(k, k_tpe, dname + '.' + k): convert_to(v, v_tpe, dname + '.' + k) for k, v in o.items()}
    if isinstance(tpe, type) and issubclass(tpe, Settings):
        assert isinstance(o, dict), 'Setting {!r} expected type {!r} got {!r} ({!r})'.format(dname, dict, type(o), o)
        return tpe(**o)
    try:
        return tpe(o)
    except (ValueError, TypeError) as e:
        raise ValueError('Setting {!r} expected type {!r} got {!r} ({!r})'.format(dname, tpe, type(o), o))
    raise ValueError('Setting {!r} decorator not a type {!r}'.format(dname, tpe))
