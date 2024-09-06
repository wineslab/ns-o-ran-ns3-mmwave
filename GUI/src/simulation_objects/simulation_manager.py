from typing import Optional

from src.simulation_objects.simulation import Simulation


class SimulationManager:
    _simulation: Optional[Simulation] = None

    @classmethod
    def get_simulation(cls) -> Simulation:
        if cls._simulation is None:
            cls._simulation = Simulation(number_of_ues=0, number_of_cells=0)
        return cls._simulation

    @classmethod
    def refresh_simulation(cls) -> Simulation:
        if cls._simulation.number_of_ues == 0 and cls._simulation.number_of_cells == 0:
            return cls._simulation
        simulation = cls.get_simulation()
        new_ues, new_cells = simulation.get_simulation_data(
            number_of_ues=simulation.number_of_ues,
            number_of_cells=simulation.number_of_cells
        )
        new_max_x, new_max_y = simulation.get_charts_max_axis_value()
        simulation.ues = new_ues
        simulation.cells = new_cells
        simulation.max_x = new_max_x
        simulation.max_y = new_max_y
        return simulation

    @classmethod
    def reset_simulation(cls):
        if cls._simulation is not None:
            cls._simulation.ues = []
            cls._simulation.cells = []
            cls._simulation.simulation_start_time = None
            cls._simulation.number_of_ues = 0
            cls._simulation.number_of_cells = 0
            cls._simulation.max_x = 6000
            cls._simulation.max_y = 6000

    @classmethod
    def delete_simulation(cls):
        cls._simulation = None
