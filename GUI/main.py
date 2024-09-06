from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from src.http.data_controller import influx_data_router, stop_simulation
from src.simulation_objects.simulation import Simulation

app = FastAPI()
app.mount("/static", StaticFiles(directory="src/static"), name="static")
app.include_router(influx_data_router)

@app.on_event('startup')
async def startup():
    await stop_simulation()