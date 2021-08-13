# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2021-08-13
### Changed
- Code and structure of several Fictibot packages to allow for more interesting exercises.

### Added
- Docker files to create a Docker image with HAROS and this example repository.

## [0.2.0] - 2021-01-25
### Added
- Several project files under [projects](./projects).
- Several scripts to run HAROS under [scripts](./scripts).
- `minimal_example` package.
- `fictibot_msgs` package.
- Subscriber to a custom message type in `fictibot_controller`.
- `state` topic publisher to `fictibot_multiplex`.
- Conditional subscribers to `fictibot_controller`.

### Changed
- Improved [README](./README.md) documentation.

## [0.1.0] - 2018-03-27
### Added
- `fictibot_driver` package.
- `fictibot_controller` package.
- `fictibot_multiplex` package.
- `index.yaml` basic project file.
