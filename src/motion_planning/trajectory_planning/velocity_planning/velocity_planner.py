from src.interface.strategy_wrapper import StrategyWrapper


class VelocityPlanner(StrategyWrapper):

    def interpolate(self, t: float):
        return self.strategy.interpolate(t)
