#!/usr/bin/env python3
"""Real Vosk two-stream timing probe. Sends silence; does not measure accuracy."""
import argparse
import asyncio
import json
import ssl
import time
from pathlib import Path

from aiohttp import ClientSession, TCPConnector, web
from vosk import Model, KaldiRecognizer
from dual_control_voice_server import (
    Config, ControlState, HEADER, PCM_BYTES, PublisherThread,
    SAMPLE_RATE, command_grammar, make_app,
)


async def probe(args):
    root = Path(__file__).resolve().parent
    started = time.monotonic()
    model = Model(str(args.model))
    grammar = command_grammar(model)
    print(json.dumps(dict(model_load_seconds=round(time.monotonic()-started, 3),
                          grammar_phrases=len(json.loads(grammar)))), flush=True)
    factory = lambda: KaldiRecognizer(model, SAMPLE_RATE, grammar)
    state = ControlState(Config())
    publisher = PublisherThread(state, dry_run=True)
    publisher.start()
    tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    tls.load_cert_chain(str(args.cert), str(args.key))
    runner = web.AppRunner(make_app(state, factory, publisher), access_log=None)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0, ssl_context=tls)
    await site.start()
    port = site._server.sockets[0].getsockname()[1]
    trust = ssl.create_default_context(cafile=str(args.ca))
    base = "https://127.0.0.1:" + str(port)
    streams, tasks = [], []
    try:
        async with ClientSession(connector=TCPConnector(ssl=trust)) as session:
            async def api(action, cid, **extra):
                async with session.post(base+"/"+action, json=dict(client_id=cid, **extra)) as response:
                    result = await response.json()
                    assert response.status == 200, result
                    return result

            for cid, role in (("probe_linear", "LINEAR"), ("probe_angular", "ANGULAR")):
                await api("connect", cid)
                await api("claim", cid, role=role)
                ws = await session.ws_connect(base+"/audio?client_id="+cid)
                stream = dict(cid=cid, ws=ws, tick=None, sequence=0)
                streams.append(stream)
                async def receive(stream=stream):
                    async for msg in stream["ws"]:
                        if msg.type == web.WSMsgType.TEXT:
                            data = json.loads(msg.data)
                            if data.get("type") == "tick":
                                stream["tick"] = data["stamp"]
                tasks.append(asyncio.create_task(receive()))

            async def heartbeat():
                while True:
                    for stream in streams:
                        await api("heartbeat", stream["cid"])
                    await asyncio.sleep(.2)
            tasks.append(asyncio.create_task(heartbeat()))
            deadline = time.monotonic()+8
            while any(s["tick"] is None for s in streams):
                assert time.monotonic() < deadline, "decoder initialization timeout"
                await asyncio.sleep(.02)

            next_frame = time.monotonic()
            for _ in range(round(args.seconds/.02)):
                for stream in streams:
                    await stream["ws"].send_bytes(
                        HEADER.pack(stream["tick"], stream["sequence"])+bytes(PCM_BYTES))
                    stream["sequence"] += 1
                next_frame += .02
                await asyncio.sleep(max(0, next_frame-time.monotonic()))
            for stream in streams:
                snapshot = state.snapshot(stream["cid"])
                assert snapshot["mic_connected"], snapshot
                assert state.clients[stream["cid"]]["audio_at"] is not None
                print(json.dumps(dict(client=stream["cid"], frames=stream["sequence"],
                                      audio=snapshot["audio"]), ensure_ascii=False), flush=True)
            assert not publisher.error
            print("PASS two real Vosk streams remained connected; silence only", flush=True)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        for stream in streams:
            await stream["ws"].close()
        await runner.cleanup()
        publisher.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--cert", type=Path, required=True)
    parser.add_argument("--key", type=Path, required=True)
    parser.add_argument("--ca", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=12)
    asyncio.run(probe(parser.parse_args()))
