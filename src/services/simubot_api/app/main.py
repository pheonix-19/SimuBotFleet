from fastapi import FastAPI
from pydantic import BaseModel
from prometheus_client import Counter, generate_latest, CONTENT_TYPE_LATEST
from fastapi.responses import Response
import os, json
from .queue import enqueue_task

app = FastAPI(title="SimuBot API")

TASKS_CREATED = Counter("simubot_tasks_created_total", "Tasks created via REST")

class Task(BaseModel):
    pick: tuple[float, float]
    drop: tuple[float, float]
    priority: int = 1

@app.post("/assign_task")
def assign_task(task: Task):
    TASKS_CREATED.inc()
    enqueue_task(json.dumps(task.dict()))
    return {"status": "queued"}

@app.get("/metrics")
def metrics():
    return Response(generate_latest(), media_type=CONTENT_TYPE_LATEST)
