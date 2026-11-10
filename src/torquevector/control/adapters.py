"""Composition adapters; numerical controller implementations stay unchanged."""

from torquevector.control.interfaces import ControllerRequest, MomentCommand


class FeedbackController:
    def __init__(self, strategy):
        self.strategy = strategy

    def command(self, request):
        value = self.strategy.demand(request.state, request.reference, request.steering_rad)
        return MomentCommand(value, {'status': 'SUCCESS', 'kind': 'feedback'})

    def demand(self, state, reference, steering_rad):
        return self.command(ControllerRequest(state, reference, steering_rad)).command_nm


class PredictiveController:
    def __init__(self, solver):
        self.solver = solver
        self.car = solver.car
        self.config = solver.config

    def command(self, request):
        if request.preview is None:
            raise ValueError('Predictive controller requires aligned preview')
        result = self.solver.solve(request.state, request.preview, request.previous_nm, request.period_s)
        return MomentCommand(
            result.command_nm,
            {'status': 'SUCCESS', 'iterations': result.iterations, 'state_excess': result.max_state_excess},
            result,
        )

    def solve(self, state, preview, previous_nm, period_s=None):
        return self.command(
            ControllerRequest(state, preview=preview, previous_nm=previous_nm, period_s=period_s)
        ).native_result
