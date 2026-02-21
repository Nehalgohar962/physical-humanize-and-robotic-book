// src/components/chatbot/loadModules.ts

export async function loadAllModules(): Promise<string> {
    const modulePaths = [
      '/docs/specs/module1/sec.md',
      '/docs/checklist/requirements.md',
      '/docs/history/module/intro.md'
    ];
  
    const contents: string[] = [];
  
    for (const path of modulePaths) {
      try {
        const res = await fetch(path);
        const text = await res.text();
        contents.push(text);
      } catch (e) {
        console.error('Failed to load module:', path);
      }
    }
  
    return contents.join('\n\n');
  }
  