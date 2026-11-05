use crate::{Params, ParseError, Token};

pub fn lex(text: &str) -> Lexer<'_> {
    Lexer { lines: lines(text) }
}

/// Every command line split into its head and the raw text after it. The
/// arguments are parsed only on request, so a consumer can skip a command it
/// does not handle whatever text follows it — as a G-code host ignores an
/// unknown `M117 Hello` without reading `Hello` as a parameter.
pub fn lines(text: &str) -> Lines<'_> {
    Lines {
        lines: text.lines().enumerate(),
    }
}

#[derive(Debug)]
pub struct Lexer<'a> {
    lines: Lines<'a>,
}

#[derive(Debug)]
pub struct Lines<'a> {
    lines: std::iter::Enumerate<std::str::Lines<'a>>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Head {
    Numbered {
        letter: u8,
        major: u32,
        minor: Option<u32>,
    },
    Extended(Box<str>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Line<'a> {
    pub line_no: u32,
    pub head: Head,
    args: &'a str,
}

impl Line<'_> {
    /// A numbered command's letter words: each a letter and a finite number.
    pub fn params(&self) -> Result<Params, ParseError> {
        let line_no = self.line_no;
        let mut params = Params::default();
        let mut seen = [false; 26];
        for tok in self.args.split_whitespace() {
            let mut tc = tok.chars();
            let Some(letter_ch) = tc.next() else { continue };
            if !letter_ch.is_ascii_alphabetic() {
                return Err(ParseError::MalformedNumber {
                    line_no,
                    text: tok.to_string().into_boxed_str(),
                });
            }
            let letter = letter_ch.to_ascii_uppercase() as u8;
            let num_str = &tok[letter_ch.len_utf8()..];
            let value: f64 = num_str.parse().map_err(|_| ParseError::MalformedNumber {
                line_no,
                text: tok.to_string().into_boxed_str(),
            })?;
            if !value.is_finite() {
                return Err(ParseError::MalformedNumber {
                    line_no,
                    text: tok.to_string().into_boxed_str(),
                });
            }
            let idx = (letter - b'A') as usize;
            if seen[idx] {
                return Err(ParseError::DuplicateParam {
                    line_no,
                    letter: letter as char,
                });
            }
            seen[idx] = true;
            params.set(letter, value);
        }
        Ok(params)
    }

    /// An extended command's `KEY=value` arguments.
    pub fn extended_args(&self) -> Result<Vec<(Box<str>, Box<str>)>, ParseError> {
        let mut args = Vec::new();
        for tok in self.args.split_whitespace() {
            let Some((key, value)) = tok.split_once('=').filter(|(key, _)| !key.is_empty()) else {
                return Err(ParseError::MalformedNumber {
                    line_no: self.line_no,
                    text: tok.to_string().into_boxed_str(),
                });
            };
            args.push((
                key.to_string().into_boxed_str(),
                value.to_string().into_boxed_str(),
            ));
        }
        Ok(args)
    }

    pub fn into_token(self) -> Result<Token, ParseError> {
        let line_no = self.line_no;
        match &self.head {
            Head::Numbered {
                letter,
                major,
                minor,
            } => Ok(Token::Command {
                letter: *letter,
                major: *major,
                minor: *minor,
                params: self.params()?,
                line_no,
            }),
            Head::Extended(name) => Ok(Token::Extended {
                name: name.clone(),
                args: self.extended_args()?,
                line_no,
            }),
        }
    }
}

fn strip_inline_comment(line: &str) -> &str {
    match line.find(';') {
        Some(idx) => &line[..idx],
        None => line,
    }
}

fn parse_head_number(s: &str) -> Option<(u32, Option<u32>)> {
    if let Some((maj, min)) = s.split_once('.') {
        let major = maj.parse::<u32>().ok()?;
        let minor = min.parse::<u32>().ok()?;
        Some((major, Some(minor)))
    } else {
        Some((s.parse::<u32>().ok()?, None))
    }
}

fn split_head(line: &str, line_no: u32) -> Result<Line<'_>, ParseError> {
    let mut chars = line.char_indices();

    let Some((_, head_char)) = chars.next() else {
        return Err(ParseError::EmptyCommand { line_no });
    };
    if !head_char.is_ascii_alphabetic() {
        return Err(ParseError::UnrecognizedHead {
            line_no,
            head: line
                .split_whitespace()
                .next()
                .unwrap_or(line)
                .to_string()
                .into_boxed_str(),
        });
    }
    let head_byte = head_char.to_ascii_uppercase() as u8;

    let after_letter_idx = chars.next().map_or(line.len(), |(i, _)| i);
    let after_letter = &line[after_letter_idx..];

    let head_number_str = after_letter
        .find(char::is_whitespace)
        .map_or(after_letter, |ws_idx| &after_letter[..ws_idx]);
    let head_len = after_letter_idx + head_number_str.len();
    let Some((major, minor)) = parse_head_number(head_number_str) else {
        let head_word = &line[..head_len];
        if head_word.len() >= 2
            && head_word
                .bytes()
                .all(|b| b.is_ascii_alphabetic() || b.is_ascii_digit() || b == b'_')
        {
            return Ok(Line {
                line_no,
                head: Head::Extended(head_word.to_ascii_uppercase().into_boxed_str()),
                args: &line[head_len..],
            });
        }
        return Err(ParseError::UnrecognizedHead {
            line_no,
            head: format!("{head_char}{head_number_str}").into_boxed_str(),
        });
    };
    Ok(Line {
        line_no,
        head: Head::Numbered {
            letter: head_byte,
            major,
            minor,
        },
        args: &line[head_len..],
    })
}

impl<'a> Iterator for Lines<'a> {
    type Item = Result<Line<'a>, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let (idx, raw) = self.lines.next()?;
            let line_no = (idx as u32).checked_add(1).expect("line count overflow");
            let no_inline = strip_inline_comment(raw.trim()).trim();
            if no_inline.is_empty() {
                continue;
            }
            return Some(split_head(no_inline, line_no));
        }
    }
}

impl Iterator for Lexer<'_> {
    type Item = Result<Token, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.lines
            .next()
            .map(|line| line.and_then(Line::into_token))
    }
}

#[cfg(test)]
mod tests;
