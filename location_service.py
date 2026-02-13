from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

app = FastAPI()

# In-memory database for locations
locations_db = []


class Location(BaseModel):
    name: str
    latitude: float
    longitude: float
    category: str  # Restaurant | Hotel | Office | Park | Other


@app.get("/")
def root():
    return {"service": "Location Service", "status": "running"}


@app.post("/locations")
def add_location(location: Location):
    """Add a new location"""
    loc_dict = location.model_dump()
    loc_dict["id"] = len(locations_db) + 1
    locations_db.append(loc_dict)
    return {"status": "success", "location": loc_dict}


@app.get("/locations")
def get_locations():
    """Get all locations"""
    return {"locations": locations_db, "count": len(locations_db)}


@app.delete("/locations/{location_id}")
def delete_location(location_id: int):
    """Delete a location"""
    global locations_db
    original_len = len(locations_db)
    locations_db = [loc for loc in locations_db if loc["id"] != location_id]
    if len(locations_db) == original_len:
        raise HTTPException(status_code=404, detail="Location not found")
    return {"status": "deleted"}


if __name__ == "__main__":
    print("Starting Location Service on port 8003...")
    uvicorn.run(app, host="0.0.0.0", port=8003)
