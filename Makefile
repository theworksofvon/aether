SHELL := /bin/bash

COMPOSE := podman compose -f compose.yaml

.PHONY: help init-env build up up-optional down restart logs ps status deploy-service start-service stop-service restart-service service-status hardware-check serial-devices tail-flight tail-edge

help:
	@echo "Aether hardware integration targets"
	@echo ""
	@echo "  make init-env         Copy .env.example to .env if .env does not exist"
	@echo "  make build            Build the core edge runtime containers"
	@echo "  make up               Start the core edge runtime with podman compose"
	@echo "  make up-optional      Start the optional sensor and vision services too"
	@echo "  make down             Stop the compose stack"
	@echo "  make restart          Restart the compose stack"
	@echo "  make logs             Follow all compose logs"
	@echo "  make ps               Show compose service status"
	@echo "  make deploy-service   Install/update the systemd unit on the Pi"
	@echo "  make start-service    Start the systemd-managed runtime"
	@echo "  make stop-service     Stop the systemd-managed runtime"
	@echo "  make restart-service  Restart the systemd-managed runtime"
	@echo "  make service-status   Show systemd status for aether.service"
	@echo "  make hardware-check   Show serial devices, compose status, and systemd status"
	@echo "  make serial-devices   List USB serial devices visible on the Pi"
	@echo "  make tail-flight      Follow logs for the flight container"
	@echo "  make tail-edge        Follow logs for the edge container"

init-env:
	@if [ -f .env ]; then \
		echo ".env already exists"; \
	else \
		cp .env.example .env; \
		echo "Created .env from .env.example"; \
	fi

build:
	./scripts/build-containers.sh

up:
	$(COMPOSE) up -d

up-optional:
	$(COMPOSE) --profile optional up -d

down:
	$(COMPOSE) down

restart: down up

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps

deploy-service:
	sudo ./scripts/deploy-service.sh

start-service:
	sudo systemctl start aether

stop-service:
	sudo systemctl stop aether

restart-service:
	sudo systemctl restart aether

service-status:
	sudo systemctl status aether --no-pager

serial-devices:
	@echo "Visible serial devices:"
	@ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No /dev/ttyUSB* or /dev/ttyACM* devices found"

tail-flight:
	$(COMPOSE) logs -f flight

tail-edge:
	$(COMPOSE) logs -f edge

hardware-check:
	@echo "== Serial devices =="
	@$(MAKE) --no-print-directory serial-devices
	@echo ""
	@echo "== Compose status =="
	@$(COMPOSE) ps
	@echo ""
	@echo "== systemd status =="
	@sudo systemctl status aether --no-pager || true
