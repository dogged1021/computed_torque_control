from src.interface.strategy_wrapper import StrategyWrapper


class PathPlanner(StrategyWrapper):

    def interpolate(self, s: float):
        return self.strategy.interpolate(s)
