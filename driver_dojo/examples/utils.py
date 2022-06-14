import yaml
import scipy
import numpy as np


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


def plot_normal(ax, name, mu, std, clip):
    dist = scipy.stats.distributions.norm
    x = np.linspace(mu - 3 * std, mu + 3 * std, endpoint=True, num=100)
    y = dist.pdf(x, loc=mu, scale=std)

    ax.plot(x, y)

    ax.axvline(x=mu,
               color='black',
               ls='--',
               lw=1,
               label='$\mathcal{N}$'f' ({mu},{std}''$^{2}$)')
    ax.axvline(x=std,
               color='black',
               ls='--',
               lw=0, )

    if clip:
        ax.axvline(x=clip[0],
                   color='black',
                   ls='--',
                   lw=1,
                   label='$x \in $'f'[{clip[0]}, {clip[1]}]')
        ax.axvline(x=clip[1],
                   color='black',
                   ls='--',
                   lw=1)
    leg = ax.legend(handlelength=0, handletextpad=0, fancybox=True, loc='upper right')
    for item in leg.legendHandles:
        item.set_visible(False)

    ax.set_xlim(mu - 3 * std, mu + 3 * std)
    ax.set_ylim(0, max(y) + max(y) / 10.0)
    ax.set_xticks([round(mu - 2 * std, 1), round(mu, 1), round(mu + 2 * std, 1)])
    ax.set_yticks([0.0, round(max(y)/2.0, 1), round(max(y) + max(y)/10.0, 1)])
    ax.set_aspect(1.0 / ax.get_data_ratio(), adjustable='box')
    ax.set_title(name)
    return ax


def plot_uniform(ax, name, mi, ma):
    dist = scipy.stats.distributions.uniform
    span = ma - mi
    x = np.linspace(mi - span / 5, ma + span / 5, endpoint=True, num=100)
    y = dist.pdf(x, loc=mi, scale=span)

    ax.plot(x, y)

    ax.axvline(x=mi,
               color='black',
               ls='--',
               lw=0,
               label=f'$\mathcal{{U}}'f'_{{[{mi}, {ma}]}}''$')

    leg = ax.legend(handlelength=0, handletextpad=0, fancybox=True, loc='upper right')
    for item in leg.legendHandles:
        item.set_visible(False)

    ax.set_xlim(mi - span / 5, ma + span / 5)
    ax.set_ylim(0, max(y) + max(y) / 10.0)
    ax.set_xticks([round(mi, 1), round(mi + span / 2, 1), round(ma, 1)])
    ax.set_yticks([0.0, round(max(y)/2.0, 1), round(max(y) + max(y)/10.0, 1)])
    ax.set_aspect(1.0 / ax.get_data_ratio(), adjustable='box')
    ax.set_title(name)
    return ax


def plot_const(ax, name, val):
    ax.axvline(x=val,
               color='black',
               ls='-',
               lw=1,
               label=f"const {val}")

    leg = ax.legend(handlelength=0, handletextpad=0, fancybox=True, loc='upper right')
    for item in leg.legendHandles:
        item.set_visible(False)

    ax.set_xlim(val - 2.0, val + 2.0)
    ax.set_ylim(0, 1.0)
    ax.set_xticks([val])
    ax.set_yticks([0.0, 1.0])
    ax.set_aspect(1.0 / ax.get_data_ratio(), adjustable='box')
    ax.set_title(name)
    return ax
