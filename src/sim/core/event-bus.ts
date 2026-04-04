/**
 * Typed synchronous event bus for intra-worker communication.
 */

type Listener<T> = (data: T) => void;

export class EventBus {
  private listeners = new Map<string, Set<Listener<unknown>>>();

  on<T>(event: string, listener: Listener<T>): () => void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    const set = this.listeners.get(event)!;
    set.add(listener as Listener<unknown>);
    return () => { set.delete(listener as Listener<unknown>); };
  }

  emit<T>(event: string, data: T): void {
    const set = this.listeners.get(event);
    if (set) {
      for (const listener of set) {
        listener(data);
      }
    }
  }

  clear(): void {
    this.listeners.clear();
  }
}
