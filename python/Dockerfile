FROM python:latest

WORKDIR /rman-python

COPY ./requirements.txt .

RUN python -m pip install -r requirements.txt

COPY . .

CMD ["python", "main.py"]