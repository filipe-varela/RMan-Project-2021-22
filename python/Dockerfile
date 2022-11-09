FROM python:latest

RUN python -m pip install --upgrade pip

WORKDIR /rman-python

COPY ./requirements.txt .

RUN python -m pip install -r requirements.txt

COPY . .

RUN python -m pip install -e .

CMD ["python", "main.py"]