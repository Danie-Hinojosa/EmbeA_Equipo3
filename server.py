# server.py
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

app = FastAPI()

# Simple in-memory storage (list of dicts)
# In production, you would use a database (SQLite, Postgres, etc.)
data_store = []

class SensorData(BaseModel):
    temperature: float
    humidity: float

@app.post("/api/data")
async def receive_data(data: SensorData):
    print(f"Received: {data}")
    data_store.append(data.dict())
    # Keep only last 10 records to keep it clean
    if len(data_store) > 10:
        data_store.pop(0)
    return {"status": "success"}

@app.get("/api/data")
async def get_data():
    return data_store

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)