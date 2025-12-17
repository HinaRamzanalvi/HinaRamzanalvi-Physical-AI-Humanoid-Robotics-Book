export interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  citations?: Citation[];
  isError?: boolean;
}

export interface Citation {
  module: string;
  chapter: string;
  section_title: string;
  source_file: string;
}

export interface ChatResponse {
  query_id: string;
  session_id: string;
  response_text: string;
  citations: Citation[];
  confidence_score: number;
  status: string;
}

export interface ChatRequest {
  query_text: string;
  query_mode: 'AskBook' | 'AskSelectedText';
  selected_text?: string;
  explanation_complexity?: 'beginner' | 'intermediate' | 'advanced';
  session_id?: string;
}