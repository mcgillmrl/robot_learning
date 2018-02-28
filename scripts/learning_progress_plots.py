import matplotlib as mpl
mpl.use('Agg')
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import pyplot as plt

def plot_rollout(rollout_fn, exp, n_exp=0, *args, **kwargs):
    fig = kwargs.get('fig')
    axarr = kwargs.get('axarr')

    ret = rollout_fn(*args)
    trajectories = m_states = None
    if len(ret) == 3:
        loss, costs, trajectories = ret
        n_samples, T, dims = trajectories.shape
    else:
        loss, m_costs, s_costs, m_states, s_states = ret
        T, dims = m_states.shape

    if fig is None or axarr is None:
        fig, axarr = plt.subplots(dims, sharex=True)

    exp_states = np.array(exp.states)
    for d in range(dims):
        axarr[d].clear()
        if trajectories is not None:
            st = trajectories[:, :, d]
            # plot predictive distribution
            for i in range(n_samples):
                axarr[d].plot(
                    np.arange(T-1), st[i, :-1], color='steelblue', alpha=0.3)
            axarr[d].plot(
                np.arange(T-1), st[:, :-1].mean(0), color='orange')
        if m_states is not None:
            axarr[d].plot(
                np.arange(T-1), m_states[:-1, d], color='steelblue',
                alpha=0.3)
            axarr[d].errorbar(
                np.arange(T-1), m_states[:-1, d],
                1.96*np.sqrt(s_states[:-1, d, d]), color='steelblue', alpha=0.3)

        total_exp = len(exp_states)
        for i in range(n_exp):
            axarr[d].plot(
                 np.arange(T-1), exp_states[total_exp - n_exp + i][1:T, d],
                 color='orange', alpha=0.3)
        # plot experience
        axarr[d].plot(
            np.arange(T-1), np.array(exp.states[-1])[1:T, d], color='red')


    return fig, axarr


def save_plots():
    with PdfPages(output_file) as pdf:
        pdf.savefig()
        plt.close()