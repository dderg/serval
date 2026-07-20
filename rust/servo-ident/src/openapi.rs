//! The single OpenAPI 3.1 description of the `serve` JSON API. Every schema
//! is generated from the same `JsonSchema`-deriving Rust type the route
//! actually serializes, so the document can never drift from the wire shape
//! without a compile-time change. `GET /api/openapi.json` hands this out
//! verbatim; the frontend and the contract tests both consume it as the one
//! source of truth. The one raw payload with no complete Rust producer type
//! (the live capture tail) is described as an open object rather than an
//! invented schema; the live tap poll is a status-discriminated union that
//! mirrors the JSON `LiveTap::poll` builds; the run manifest is a schema-only
//! contract over the Python-authored fields the dashboard reads.

use schemars::{generate::SchemaSettings, JsonSchema, SchemaGenerator};
use serde_json::{json, Map, Value};

use crate::demo::DriveStatePayload;
use crate::results::{PlotSeries, Results};
use crate::serve::{DeleteResponse, LiveStatus, NoteBody, NoteResponse, RunPath, RunSummary};
use crate::strain::StrainMap;

fn generator() -> SchemaGenerator {
    let mut settings = SchemaSettings::draft2020_12();
    settings.definitions_path = "/components/schemas".into();
    settings.meta_schema = None;
    SchemaGenerator::new(settings)
}

fn schema_ref<T: JsonSchema>(generator: &mut SchemaGenerator) -> Value {
    generator.subschema_for::<T>().to_value()
}

fn json_content(schema: Value) -> Value {
    json!({ "application/json": { "schema": schema } })
}

fn ok(description: &str, schema: Value) -> Value {
    json!({ "description": description, "content": json_content(schema) })
}

fn error_ref(name: &str) -> Value {
    json!({ "$ref": format!("#/components/responses/{name}") })
}

fn free_form(description: &str) -> Value {
    json!({
        "type": "object",
        "additionalProperties": true,
        "description": description,
    })
}

fn live_tap_schema() -> Value {
    let u32_num = json!({ "type": "integer", "format": "uint32", "minimum": 0 });
    let u64_num = json!({ "type": "integer", "format": "uint64", "minimum": 0 });
    let i64_array = json!({ "type": "array", "items": { "type": "integer", "format": "int64" } });
    let timing = json!({
        "oneOf": [
            {
                "type": "object",
                "required": ["skips", "late_frames", "lateness_ns"],
                "properties": {
                    "skips": u32_num.clone(),
                    "late_frames": u32_num.clone(),
                    "lateness_ns": { "type": "integer", "format": "int32" },
                },
                "additionalProperties": false,
            },
            { "type": "null" },
        ]
    });
    let drive_series = json!({
        "type": "object",
        "required": ["ferr", "torque", "target", "pos"],
        "properties": {
            "ferr": i64_array.clone(),
            "torque": i64_array.clone(),
            "target": i64_array.clone(),
            "pos": i64_array.clone(),
        },
        "additionalProperties": false,
    });
    json!({
        "description": "Live tap poll result, discriminated by status: connecting \
                        while the tap session is opening, unreachable when the last \
                        connect attempt failed, streaming once samples flow.",
        "oneOf": [
            {
                "type": "object",
                "title": "LiveTapConnecting",
                "required": ["status"],
                "properties": { "status": { "type": "string", "const": "connecting" } },
                "additionalProperties": false,
            },
            {
                "type": "object",
                "title": "LiveTapUnreachable",
                "required": ["status", "reason"],
                "properties": {
                    "status": { "type": "string", "const": "unreachable" },
                    "reason": { "type": "string" },
                },
                "additionalProperties": false,
            },
            {
                "type": "object",
                "title": "LiveTapStreaming",
                "required": [
                    "status", "fs_hz", "cycle_ns", "drive_names",
                    "counts_per_mm", "next_cycle", "timing"
                ],
                "properties": {
                    "status": { "type": "string", "const": "streaming" },
                    "fs_hz": { "type": "number", "format": "double" },
                    "cycle_ns": u64_num.clone(),
                    "drive_names": { "type": "array", "items": { "type": "string" } },
                    "counts_per_mm": {
                        "type": "array",
                        "items": { "type": "number", "format": "double" }
                    },
                    "next_cycle": u64_num.clone(),
                    "timing": timing,
                    "first_cycle": { "oneOf": [u64_num.clone(), { "type": "null" }] },
                    "stride": u64_num.clone(),
                    "drives": { "type": "object", "additionalProperties": drive_series },
                    "moving": { "type": "array", "items": { "type": "boolean" } },
                },
                "additionalProperties": false,
            },
        ],
        "discriminator": { "propertyName": "status" },
    })
}

fn path_name_param(description: &str) -> Value {
    json!({
        "name": "name",
        "in": "path",
        "required": true,
        "schema": { "type": "string" },
        "description": description,
    })
}

fn query_param(name: &str, schema: Value, description: &str) -> Value {
    json!({
        "name": name,
        "in": "query",
        "required": false,
        "schema": schema,
        "description": description,
    })
}

fn manifest_schemas() -> Vec<(&'static str, Value)> {
    vec![
        (
            "Manifest",
            json!({
                "type": "object",
                "required": ["experiment", "steps"],
                "properties": {
                    "experiment": { "type": "string" },
                    "command": { "type": ["string", "null"] },
                    "tag": { "type": ["string", "null"] },
                    "axis": { "type": ["string", "null"] },
                    "stroke_plan": {
                        "anyOf": [
                            { "$ref": "#/components/schemas/StrokePlan" },
                            { "type": "null" },
                        ],
                    },
                    "steps": {
                        "type": "array",
                        "items": { "$ref": "#/components/schemas/ManifestStep" },
                    },
                    "motors": {
                        "type": ["array", "null"],
                        "items": { "$ref": "#/components/schemas/ManifestMotor" },
                    },
                    "ambient": {
                        "anyOf": [
                            { "$ref": "#/components/schemas/ManifestAmbient" },
                            { "type": "null" },
                        ],
                    },
                },
                "description": "Run manifest as written by the Python capture tooling: a typed \
                                view of the fields the dashboard reads, with additional producer \
                                fields permitted.",
            }),
        ),
        (
            "StrokePlan",
            json!({
                "type": "object",
                "properties": {
                    "speed": { "type": ["number", "null"] },
                    "accel": { "type": ["number", "null"] },
                    "iterations": { "type": ["number", "null"] },
                    "line_spacing": { "type": ["number", "null"] },
                    "x_start": { "type": ["number", "null"] },
                    "x_end": { "type": ["number", "null"] },
                    "y_start": { "type": ["number", "null"] },
                    "y_end": { "type": ["number", "null"] },
                    "dwell_ms": { "type": ["number", "null"] },
                    "zero_sync": { "type": ["boolean", "null"] },
                    "belt": { "type": ["string", "null"] },
                    "freq_start": { "type": ["number", "null"] },
                    "freq_end": { "type": ["number", "null"] },
                    "amplitude": { "type": ["number", "null"] },
                    "duration": { "type": ["number", "null"] },
                    "ramp": { "type": ["number", "null"] },
                    "cruise_ms": { "type": ["number", "null"] },
                    "speeds": { "type": ["array", "null"], "items": { "type": "number" } },
                },
            }),
        ),
        (
            "ManifestStep",
            json!({
                "type": "object",
                "required": ["name", "swept"],
                "properties": {
                    "name": { "type": "string" },
                    "swept": {
                        "type": ["object", "null"],
                        "additionalProperties": { "type": "number" },
                    },
                },
            }),
        ),
        (
            "ManifestMotor",
            json!({
                "type": "object",
                "required": ["name", "counts_per_mm"],
                "properties": {
                    "name": { "type": "string" },
                    "counts_per_mm": { "type": ["number", "null"] },
                },
            }),
        ),
        (
            "ManifestAmbient",
            json!({
                "type": "object",
                "properties": {
                    "journal_params": {
                        "type": ["object", "null"],
                        "additionalProperties": {
                            "type": "object",
                            "additionalProperties": { "type": ["number", "string"] },
                        },
                    },
                    "notches": {
                        "type": ["object", "null"],
                        "additionalProperties": {
                            "type": "object",
                            "additionalProperties": { "$ref": "#/components/schemas/NotchStateValue" },
                        },
                    },
                },
            }),
        ),
        (
            "NotchStateValue",
            json!({
                "anyOf": [
                    { "type": "object", "additionalProperties": { "type": ["number", "string"] } },
                    { "type": "number" },
                    { "type": "string" },
                ],
            }),
        ),
    ]
}

/// The deterministic OpenAPI 3.1 document covering every JSON `/api` route
/// plus `/api/openapi.json` itself. Component schemas are shared and
/// referenced; serializing the returned value twice yields byte-identical
/// output.
#[must_use]
pub fn document() -> Value {
    let mut generator = generator();

    let runs_list = json!({
        "type": "array",
        "items": schema_ref::<RunSummary>(&mut generator),
    });
    let drive_state = schema_ref::<DriveStatePayload>(&mut generator);
    let live_status = schema_ref::<LiveStatus<'static>>(&mut generator);
    let results = schema_ref::<Results>(&mut generator);
    let plot_series = schema_ref::<PlotSeries>(&mut generator);
    let run_path = schema_ref::<RunPath>(&mut generator);
    let strain = schema_ref::<StrainMap>(&mut generator);
    let note_request = schema_ref::<NoteBody>(&mut generator);
    let note_response = schema_ref::<NoteResponse<'static>>(&mut generator);
    let delete_response = schema_ref::<DeleteResponse<'static>>(&mut generator);

    let openapi_doc = free_form("This OpenAPI 3.1 document.");
    let manifest = json!({ "$ref": "#/components/schemas/Manifest" });
    let live_tail = free_form(
        "Incremental slice of an in-progress capture (records plus the next \
         byte offset); an ad-hoc shape with no complete Rust producer type.",
    );
    let live_tap = live_tap_schema();

    let mut paths = Map::new();

    paths.insert(
        "/api/openapi.json".into(),
        json!({
            "get": {
                "summary": "This OpenAPI 3.1 document.",
                "responses": { "200": ok("The OpenAPI document.", openapi_doc) },
            }
        }),
    );

    paths.insert(
        "/api/runs".into(),
        json!({
            "get": {
                "summary": "List runs, newest first.",
                "responses": {
                    "200": ok("Run summaries, newest first.", runs_list),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/drive_state".into(),
        json!({
            "get": {
                "summary": "Tuning-panel drive state with a fresh age_s field.",
                "responses": {
                    "200": ok("drive_state.json plus an added age_s field.", drive_state),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/live".into(),
        json!({
            "get": {
                "summary": "Newest flat capture with its current size and age.",
                "responses": {
                    "200": ok("Live capture status.", live_status),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/live/{name}".into(),
        json!({
            "get": {
                "summary": "Incremental tail of an in-progress capture.",
                "parameters": [
                    path_name_param("Capture file name."),
                    query_param(
                        "offset",
                        json!({ "type": "string" }),
                        "Byte offset to resume from, or \"end\" for the aligned EOF.",
                    ),
                ],
                "responses": {
                    "200": ok("Capture tail slice.", live_tail),
                    "400": error_ref("BadRequest"),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/live_tap".into(),
        json!({
            "get": {
                "summary": "Poll the live tap ring since a cycle counter.",
                "parameters": [
                    query_param(
                        "since_cycle",
                        json!({ "type": "integer", "format": "uint64", "minimum": 0 }),
                        "Only return cycles strictly newer than this counter.",
                    ),
                ],
                "responses": {
                    "200": ok("Live tap snapshot.", live_tap),
                    "400": error_ref("BadRequest"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/manifest".into(),
        json!({
            "get": {
                "summary": "Raw run manifest.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("The run manifest, verbatim.", manifest),
                    "404": error_ref("NotFound"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/results".into(),
        json!({
            "get": {
                "summary": "Analysis results for a run.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("Analysis results.", results.clone()),
                    "404": error_ref("NotFound"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/plot_series".into(),
        json!({
            "get": {
                "summary": "Plot series for a run.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("Plot series.", plot_series),
                    "404": error_ref("NotFound"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/path".into(),
        json!({
            "get": {
                "summary": "Full-resolution toolpath for a run.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("Toolpath steps.", run_path),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/strain".into(),
        json!({
            "get": {
                "summary": "Strain map for a strain_map run.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("Strain map.", strain),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/analyze".into(),
        json!({
            "post": {
                "summary": "Analyze a run on demand, returning its results.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("Analysis results.", results),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}/note".into(),
        json!({
            "post": {
                "summary": "Set or clear a run's note.",
                "parameters": [path_name_param("Run directory name.")],
                "requestBody": {
                    "required": true,
                    "content": json_content(note_request),
                },
                "responses": {
                    "200": ok("The stored note (empty when cleared).", note_response),
                    "400": error_ref("BadRequest"),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    paths.insert(
        "/api/runs/{name}".into(),
        json!({
            "delete": {
                "summary": "Delete a run directory and everything in it.",
                "parameters": [path_name_param("Run directory name.")],
                "responses": {
                    "200": ok("The deleted run name.", delete_response),
                    "404": error_ref("NotFound"),
                    "500": error_ref("ServerError"),
                },
            }
        }),
    );

    let mut schemas = generator.definitions().clone();
    schemas.insert(
        "ApiError".into(),
        json!({
            "type": "object",
            "required": ["error"],
            "properties": { "error": { "type": "string" } },
            "additionalProperties": false,
            "description": "JSON error body: a single human-readable reason.",
        }),
    );
    for (name, schema) in manifest_schemas() {
        schemas.insert(name.into(), schema);
    }

    let responses = json!({
        "NotFound": {
            "description": "The requested resource does not exist.",
            "content": json_content(json!({ "$ref": "#/components/schemas/ApiError" })),
        },
        "BadRequest": {
            "description": "A query parameter or request body could not be parsed.",
            "content": { "text/plain": { "schema": { "type": "string" } } },
        },
        "ServerError": {
            "description": "The server failed to read, parse, or compute the response.",
            "content": { "text/plain": { "schema": { "type": "string" } } },
        },
    });

    json!({
        "openapi": "3.1.0",
        "info": {
            "title": "servo-cal serve API",
            "version": env!("CARGO_PKG_VERSION"),
        },
        "paths": Value::Object(paths),
        "components": {
            "schemas": Value::Object(schemas),
            "responses": responses,
        },
    })
}
