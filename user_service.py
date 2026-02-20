from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

app = FastAPI()

# In-memory database for users
users_db = []

class User(BaseModel):
    name: str
    email: str

@app.get("/")
def root():
    return {"service": "User Service", "status": "running"}

@app.post("/users")
def add_user(user: User):
    """Add a new user"""
    user_dict = user.model_dump()
    user_dict["id"] = len(users_db) + 1
    users_db.append(user_dict)
    return {"status": "success", "user": user_dict}

@app.get("/users")
def get_users():
    """Get all users"""
    return {"users": users_db, "count": len(users_db)}

@app.get("/users/{user_id}")
def get_user(user_id: int):
    """Get a specific user"""
    for user in users_db:
        if user["id"] == user_id:
            return {"user": user}
    raise HTTPException(status_code=404, detail="User not found")

@app.delete("/users/{user_id}")
def delete_user(user_id: int):
    """Delete a user"""
    global users_db
    users_db = [u for u in users_db if u["id"] != user_id]
    return {"status": "deleted"}

if __name__ == "__main__":
    print("Starting User Service on port 8011...")
    uvicorn.run(app, host="0.0.0.0", port=8011)
