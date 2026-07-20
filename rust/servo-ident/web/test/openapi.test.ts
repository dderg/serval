import { expect, test } from "bun:test";
import { unlinkSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";

const webRoot = new URL("..", import.meta.url).pathname;
const spec = join(webRoot, "openapi.json");
const committed = join(webRoot, "src/api/openapi.generated.ts");
const generator = join(webRoot, "node_modules/.bin/openapi-typescript");

test("committed OpenAPI types match the local generator output", async () => {
  const temp = join(tmpdir(), `servo-openapi-${process.pid}-${Date.now()}.ts`);
  try {
    const proc = Bun.spawn([generator, spec, "--output", temp], {
      cwd: webRoot,
      stdout: "pipe",
      stderr: "pipe",
    });
    const code = await proc.exited;
    expect(code).toBe(0);
    const fresh = await Bun.file(temp).bytes();
    const shipped = await Bun.file(committed).bytes();
    expect(fresh).toEqual(shipped);
  } finally {
    try {
      unlinkSync(temp);
    } catch {}
  }
});
