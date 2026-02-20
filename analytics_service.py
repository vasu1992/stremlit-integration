from fastapi import FastAPI
from datetime import datetime
import uvicorn

app = FastAPI()

# In-memory database for events
events_db = []

@app.get("/")
def root():
    return {"service": "Analytics Service", "status": "running"}

@app.post("/log")
def log_event(event_name: str, value: float):
    """Log a new event"""
    event = {
        "id": len(events_db) + 1,
        "name": event_name,
        "value": value,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    events_db.append(event)
    return {"status": "logged", "event": event}

@app.get("/events")
def get_events():
    """Get all events"""
    return {"events": events_db, "count": len(events_db)}

@app.get("/summary")
def get_summary():
    """Get analytics summary"""
    if not events_db:
        return {
            "total_events": 0,
            "average_value": 0,
            "min_value": 0,
            "max_value": 0
        }
    
    values = [event["value"] for event in events_db]
    return {
        "total_events": len(events_db),
        "average_value": round(sum(values) / len(values), 2),
        "min_value": min(values),
        "max_value": max(values)
    }

@app.delete("/events")
def clear_events():
    """Clear all events"""
    global events_db
    events_db = []
    return {"status": "cleared"}

if __name__ == "__main__":
    print("Starting Analytics Service on port 8012...")
    uvicorn.run(app, host="0.0.0.0", port=8012)
