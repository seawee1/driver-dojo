import yaml


def read_yaml(path):
    yaml_dict = None
    with open(path, "r") as f:
        try:
            yaml_dict = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)
            exit()
    return yaml_dict


class DotDict(dict):
    def __getattr__(self, item):
        try:
            return self[item]
        except KeyError as e:
            raise AttributeError from e
        __setattr__ = dict.__setitem__
        __delattr__ = dict.__delitem__
