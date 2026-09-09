"""Analysis-only policy identities; ordinary production PID path stays intact."""
import pytest

from test_activation_step import controller, targets, outputs
from analysis.activation_comparison import policy


def test_proportional_priority_preserves_pd_difference():
    app=controller()
    targets(app,16)
    app.pids['kitchen'].set_integral(2.1)
    targets(app,19.5)
    with policy('proportional'):
        result=outputs(app)
    pd=lambda r: app.pids[r].p_term+app.pids[r].deriv.get()
    assert result['study']==pytest.approx(2)
    assert result['study']-result['kitchen']==pytest.approx(pd('study')-pd('kitchen'))
    assert app.pids['study'].i_term==pytest.approx(app.pids['kitchen'].i_term)


@pytest.mark.parametrize('step,error',[(.1,1.3),(.5,1.3),(-2,1.3),(2,.49)])
def test_proportional_is_inert_without_qualifying_trigger(step,error):
    apps=[controller(),controller()]
    for app in apps:
        targets(app,16)
        app.pids['kitchen'].set_integral(2.1)
        targets(app,16+step)
    with policy('original'):
        baseline=outputs(apps[0],error)
    with policy('proportional'):
        candidate=outputs(apps[1],error)
    assert candidate==baseline


def test_policy_restores_method_even_after_failure():
    from analysis.activation_comparison import actrl
    original=actrl.Actrl._calculate_pid_outputs
    with pytest.raises(RuntimeError):
        with policy('proportional'):
            raise RuntimeError()
    assert actrl.Actrl._calculate_pid_outputs is original


def test_original_and_match_reproduce_activation_difference():
    results={}
    for arm in ['original','match','proportional']:
        app=controller()
        targets(app,16)
        app.pids['kitchen'].set_integral(2.1)
        targets(app,19.5)
        with policy(arm):
            results[arm]=outputs(app)
    assert results['original']['study']<results['match']['study']
    assert results['match']['study']==results['proportional']['study']==pytest.approx(2)
    assert results['proportional']['kitchen']<results['match']['kitchen']
