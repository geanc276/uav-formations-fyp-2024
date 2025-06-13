FROM ubuntu
RUN apt-get update
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip

RUN pip3 install Flask

COPY server.py server.py

CMD python3 -m flask --app server run --host 0.0.0.0