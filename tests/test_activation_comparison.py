"""Analysis policy identities, including historical alternatives."""
import pytest

from test_activation_step import controller, targets, outputs
from analysis.activation_comparison import actrl, policy


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


def test_proportional_simultaneous_steps_share_one_integral_reference():
    app=controller()
    requested={'heat':{'study':16,'bed_2':16,'kitchen':20},'cool':{}}
    app._update_room_targets({r:20 for r in app.pids},requested)
    app.pids['kitchen'].set_integral(2.1)
    requested['heat'].update(study=19.5,bed_2=19.5)
    app._update_room_targets({r:20 for r in app.pids},requested)
    errors={'heat':{'study':1.3,'bed_2':.8,'kitchen':-.1}}
    with policy('proportional'):
        result=app._calculate_pid_outputs(errors)
    assert result==pytest.approx({'study':2,'bed_2':1.5,'kitchen':.6})


def test_proportional_cancel_closes_then_allows_fresh_reactivation():
    app=controller()
    targets(app,16)
    app.pids['kitchen'].set_integral(2.1)
    targets(app,19.5)
    with policy('proportional'):
        outputs(app)
        targets(app,16)
        assert actrl.damper_share(outputs(app,-2.2)['study'])==0
        targets(app,19.5)
        assert actrl.damper_share(outputs(app)['study'])==pytest.approx(1)
