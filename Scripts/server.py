import asyncio
import websockets

CLIENTS = set()

async def handler(websocket):
    CLIENTS.add(websocket)
    # print(CLIENTS)
    try:
        async for message in websocket:
            broadcast(message)
    finally:
        CLIENTS.remove(websocket)

async def send(websocket, message):
    try:
        await websocket.send(message)
    except websockets.ConnectionClosed:
        pass

def broadcast(message):
    for websocket in CLIENTS:
        print(message)
        asyncio.create_task(send(websocket, message))

async def main():
    async with websockets.serve(handler, "localhost", 7000):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())