import matplotlib.pyplot as plt
import numpy as np

epsilon_ = 0.2
min_ray_length_ = 0.2
field_of_view_ = 0.4
p_miss_ = 0.3
p_hit_ = 0.7
p_init_ = 0.5


def get_occ_prob(R, x, angle):
    """
    Get occupancy probability on one single line based on angle and distance
    :param R: sonar range read (scalar)
    :param angle: angle between the target line and centric sonar ray
    :param x: an arbitrary array represents equidistributed points in x axis
    :return res: occupancy probability for each x
    """
    angle_decay = max(1 - (2 * angle / field_of_view_)**2, 0.0)
    decayed_p_miss_ = p_init_ - (p_init_ - p_miss_) * angle_decay
    decayed_p_hit_ = p_init_ + (p_hit_ - p_init_) * angle_decay
    res = np.ones_like(x) * p_init_
    for i in range(x.shape[0]):
        length = x[i]
        if length >= min_ray_length_ and length <= R - epsilon_:
            res[i] = decayed_p_miss_ + min((length / (R - epsilon_))**2, 1.0) * (p_init_ - decayed_p_miss_)
        elif length > R - epsilon_ and length <= R + epsilon_:
            res[i] = decayed_p_hit_ - min(((length - R) / epsilon_)**2, 1.0) * (decayed_p_hit_ - p_init_)
    return res


def plot_occ_prob(ax, R, x, p, angle):
    """
    Plot occupied probability
    """
    vis_min_y, vis_max_y = 0.2, 0.8
    ax.plot(x, p, 'b-')
    ax.plot(x, np.ones_like(x) * p_init_, linestyle='--', color='grey')
    ax.plot(R * np.ones_like(np.arange(vis_min_y, vis_max_y, 0.01)), np.arange(vis_min_y, vis_max_y, 0.01), linestyle='--', color='grey')
    ax.set_title("ray angle = {:.3f} (rad)".format(angle))
    ax.set_xlabel("distance from sonar (m)")
    ax.set_ylabel("occupied probability")
    ax.set_ylim([vis_min_y, vis_max_y])
    ax.annotate("range", (R, p[x==R]), (R + 0.01, p[x==R] + 0.01))
    ax.plot(R, p[x==R], marker="o", markersize=5, markeredgecolor="red", markerfacecolor="red")


if __name__ == "__main__":
    """
    Visualize the probability density distribution of one sonar read
    Sonar field of view is set to be 0.4 rad
    """
    x = np.arange(0.0, 4.0, 0.01)
    R = 3
    
    fig, axs = plt.subplots(2, 2, sharex=False, sharey=False)
    fig.tight_layout()

    angle = 0
    p_1 = get_occ_prob(R, x, angle)
    plot_occ_prob(axs[0, 0], R, x, p_1, angle)

    angle = 0.2 / 3
    p_2 = get_occ_prob(R, x, angle)
    plot_occ_prob(axs[0, 1], R, x, p_2, angle)

    angle = 0.2 / 3 * 2
    p_3 = get_occ_prob(R, x, angle)
    plot_occ_prob(axs[1, 0], R, x, p_3, angle)

    angle = 0.2
    p_4 = get_occ_prob(R, x, angle)
    plot_occ_prob(axs[1, 1], R, x, p_4, angle)

    plt.show()
