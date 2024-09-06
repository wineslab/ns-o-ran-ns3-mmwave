import os
import subprocess
from dataclasses import asdict

import paramiko
from fastapi import APIRouter, Request, Depends
from fastapi.responses import JSONResponse
from fastapi.templating import Jinja2Templates

from src.simulation_objects.simulation import Simulation
from src.simulation_objects.simulation_manager import SimulationManager

influx_data_router = APIRouter()
templates = Jinja2Templates(directory="src/templates")


def get_simulation() -> Simulation:
    return SimulationManager.get_simulation()


@influx_data_router.get("/")
async def root(request: Request, simulation: Simulation = Depends(get_simulation)):
    return templates.TemplateResponse(
        "chart.html",
        {
            "request": request,
            "ues": simulation.ues,
            "cells": simulation.cells,
            "chart_dimensions": (simulation.max_x, simulation.max_y),
        },
    )


@influx_data_router.get("/refresh-data")
async def refresh_data(request: Request, simulation: Simulation = Depends(get_simulation)):
    SimulationManager.refresh_simulation()
    updated_simulation = SimulationManager.get_simulation()
    return {
        "ues": [asdict(ue) for ue in updated_simulation.ues],
        "cells": [asdict(cell) for cell in updated_simulation.cells],
        "max_x_max_y": (updated_simulation.max_x, updated_simulation.max_y),
    }


@influx_data_router.post("/start_simulation")
async def start_simulation(request: Request):
    form_data = await request.json()
    SimulationManager.reset_simulation()
    remote_host = os.getenv('NS3_HOST')
    if not remote_host:
        print("NS3_HOST environment variable is not set.")
        return
    fields = [
        "e2TermIp",
        "hoSinrDifference",
        "indicationPeriodicity",
        "simTime",
        "KPM_E2functionID",
        "RC_E2functionID",
        "N_MmWaveEnbNodes",
        #"N_LteEnbNodes",
        "N_Ues",
        "CenterFrequency",
        "Bandwidth",
        "N_AntennasMcUe",
        "N_AntennasMmWave",
        "IntersideDistanceUEs",
        "IntersideDistanceCells"
    ]
    if form_data.get('flexric') == 'true':
        arguments = '--E2andLogging=1 '
    else:
        arguments = '--enableE2FileLogging=1 '
    for field in fields:
        value = form_data.get(field)
        if value is not None:
            arguments += f"--{field}={value} "
        elif value is None and field == 'simTime':
            arguments += f"--simTime=100 "
    command = f'./waf --run "scratch/scenario-zero-with_parallel_loging.cc {arguments}"'
    command = f'curl -X POST -d \'{command}\' http://{remote_host}:38866'
    try:
        print(f'Sending start command: {command}')
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print("Response from server:")
        print(result.stdout)
    except Exception as e:
        print(f"An error occurred: {e}")
    number_of_ues = int(form_data.get('N_Ues', 2))
    number_of_cells = int(form_data.get('N_LteEnbNodes', 1)) + int(form_data.get('N_MmWaveEnbNodes', 4))
    SimulationManager._simulation = Simulation(number_of_ues, number_of_cells)



@influx_data_router.post("/reset_simulation")
async def reset_simulation():
    SimulationManager.reset_simulation()
    return {"message": "Simulation reset"}


@influx_data_router.post("/stop_simulation")
async def stop_simulation():
    remote_host = os.getenv('NS3_HOST')
    if not remote_host:
        print("NS3_HOST environment variable is not set.")
        return

    command = f"curl -X POST -d 'scenario-zero-with_parallel_loging' http://{remote_host}:38867"

    try:
        print(f'Sending stop command: {command}')
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print("Response from server:")
        print(result.stdout)
    except Exception as e:
        print(f"An error occurred: {e}")


